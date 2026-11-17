"""Geometry, wind, terrain and paired search checks for the flyable replay."""
import math
from types import SimpleNamespace
import unittest

import numpy as np

import flyable_mission as f
from test_terrain_sim import flat
from turn_path import ground_radius, left, rounded_path


def tangent(piece, point):
    if piece.center is None:
        return (piece.end - piece.start) / piece.length
    radius = np.linalg.norm(point - piece.center)
    return np.sign(piece.sweep) * left(point - piece.center) / radius


class TurnPathTests(unittest.TestCase):
    def check_path(self, path, radius):
        for p, q in zip(path, path[1:]):
            np.testing.assert_allclose(p.end, q.start, atol=1e-7)
            np.testing.assert_allclose(tangent(p, p.end), tangent(q, q.start), atol=1e-7)
        for p in path:
            xy, directions, distance = p.sample(15)
            np.testing.assert_allclose(xy[[0, -1]], [p.start, p.end], atol=1e-7)
            np.testing.assert_allclose(np.linalg.norm(directions, axis=1), 1)
            self.assertLessEqual(np.diff(distance).max(), 15 + 1e-7)
            if p.center is not None:
                np.testing.assert_allclose(np.linalg.norm(xy - p.center, axis=1), radius)

    def test_quarter_circle(self):
        legs = [((0, 0), (200, 0)), ((200, 0), (200, 200))]
        path, fillets, loops = rounded_path(legs, 60)
        self.check_path(path, 60)
        self.assertEqual((fillets, loops), (1, 0))
        self.assertAlmostEqual(sum(p.length for p in path), 280 + 30 * math.pi)
        self.assertEqual([p.source for p in path], [0, None, 1])

    def test_short_stagger_and_reversal(self):
        legs = [((0, 0), (300, 0)), ((300, 0), (300, 65)), ((300, 65), (600, 65)),
                ((600, 65), (300, 65))]
        path, fillets, loops = rounded_path(legs, 60)
        self.check_path(path, 60)
        self.assertEqual((fillets, loops), (1, 2))
        self.assertEqual([p.source for p in path if p.source is not None], [0, 1, 2, 3])

    def test_reversal_has_analytic_length(self):
        legs = [((0, 0), (100, 0)), ((100, 0), (0, 0))]
        path, _, _ = rounded_path(legs, 60)
        self.check_path(path, 60)
        self.assertAlmostEqual(sum(p.length for p in path[1:-1]), 60 * 7 * math.pi / 3)

    def test_random_dense_routes(self):
        rng = np.random.default_rng(803)
        for _ in range(30):
            xy = np.cumsum(rng.uniform(-220, 220, (20, 2)), axis=0)
            path, _, _ = rounded_path(list(zip(xy, xy[1:])), 90)
            self.check_path(path, 90)

    def test_tailwind_radius_and_crosswind_envelope(self):
        angles = np.linspace(0, math.tau, 1441)
        headings = np.column_stack((np.cos(angles), np.sin(angles)))
        radius = ground_radius(18, 10, f.t.ms.BANK, f.t.ms.G)
        self.assertAlmostEqual(radius, 138.47, places=1)
        for wind in (0, 2, 4, 6, 8, 10):
            gs, bank = f.ground_kinematics(18, headings, 1 / radius, (wind, 0))
            self.assertAlmostEqual(gs.min(), 18 - wind)
            self.assertAlmostEqual(bank.max(), (18 + wind)**2 / (f.t.ms.G * radius))
        self.assertAlmostEqual(bank.max(), math.tan(f.t.ms.BANK))

    def test_off_track_ridge_and_grid_boundary(self):
        legs = [((0, 0), (600, 0)), ((600, 0), (600, 600))]
        dem = flat()
        # A hill at (555, 45) lies inside the fillet, away from both original legs.
        dem[498, 518] += 400
        route = f.terrain_route(legs, dem, 1000, 18, 10)
        arc = next(i for i, p in enumerate(route.pieces) if p.center is not None)
        a, b = route.offsets[arc:arc + 2]
        arc_xy = route.pieces[arc].sample(f.STEP)[0]
        arc_ground = f.t.dem_at(dem, *arc_xy.T)
        self.assertGreater(arc_ground.max(), 1200)
        self.assertLessEqual(f.t.profile(dem, legs)[0][1].max(), 1120)
        self.assertTrue(np.all(route.altitude[a:b + 1] >= arc_ground + f.t.H_SEARCH - 1e-8))
        gradient = route.dz / route.ds
        self.assertLessEqual(gradient.max(), f.t.CLIMB_GRAD + 1e-10)
        self.assertGreaterEqual(gradient.min(), -f.t.DESC_GRAD - 1e-10)
        clipped = f.terrain_route([*legs, ((600, 600), (16000, 600))], dem, 1000, 18, 10)
        self.assertEqual(clipped.end_reason, "map edge")

    def test_batched_return_matches_scalar(self):
        dem = flat()
        dem[496:504, 505:510] += 350
        ends = np.array([[0, 0], [600, 0], [630, 450], [300, -7]])
        altitude = np.array([1100, 1120, 1150, 1160])
        expected = [f.t.return_climb(dem, end, z, 1120) for end, z in zip(ends, altitude)]
        np.testing.assert_allclose(f.return_ascents(dem, ends, altitude, 1000), expected, atol=1e-10)

    def test_launch_find_and_reserve_on_straight(self):
        dem = flat()
        ac = f.t.site_fleet(1120, include_competitors=False)[0]
        route = f.build_route(ac, [((0, 0), (4000, 0))], dem, 1000, 10, 1120)
        hiker = SimpleNamespace(on_map=True, tx=2000, ty=0, z=1000, draws=np.zeros(1))
        result = f.Flight(ac, route).replay(hiker, np.zeros(2), (0, .9), dem, 1000)
        self.assertEqual(result["source"], 0)
        self.assertAlmostEqual(result["t_find"], (2000 + 120 / f.t.CLIMB_GRAD) / 18)
        self.assertAlmostEqual(result["searched_km"], 2)
        ac.energy = 1
        result = f.Flight(ac, route).replay(hiker, np.zeros(2), (0, .9), dem, 1000)
        self.assertEqual(result["cutoff"], "reserve")
        self.assertFalse(result["found"])


if __name__ == "__main__":
    unittest.main()
