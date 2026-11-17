"""Real-terrain round, step 1: fetch elevation and trail/road/stream maps around real SAR trailheads.

Sites: busy US national-park search areas (NPS SAR counts 2018-20, Outforia records request:
Yosemite 732, Sequoia & Kings Canyon 503, Rocky Mountain 341, Olympic 204, Mount Rainier 146),
plus the Great Smoky Mountains (temperate forest, the most-visited park) and New Hampshire's
White Mountains (the busiest SAR area in the US Northeast). Grand Canyon (785) is left out: it is
desert canyon, not the ISRID "temperate mountains" profile used for where hikers are found.

Elevation: AWS Open Data terrain tiles (Terrarium PNG, zoom 12, ~30 m), no key needed.
Map features: OpenStreetMap via the Overpass API (ODbL; (c) OpenStreetMap contributors).
Tree canopy: USFS/MRLC NLCD Tree Canopy Cover (percent canopy per 30 m cell, CONUS, 2021), MRLC
GeoServer WCS 1.0.0, public domain. Saved as canopy.npy (fraction 0-1; no-data cells set to 0).

Each site is a 30 x 30 km square centred on the trailhead, on a local east/north grid of 30 m cells.
Run:  uv run python scripts/terrain_fetch.py
Writes: data/terrain/<site>/dem.npy, osm.json, meta.json (cached; not in git)
"""
from io import BytesIO
from pathlib import Path
import json
import math
import time

import numpy as np
import requests
from PIL import Image
from scipy.ndimage import map_coordinates

ROOT = Path(__file__).resolve().parents[1]
DATA = ROOT / "data" / "terrain"
HALF = 15000.0            # m
CELL = 30.0               # m
ZOOM = 12
UA = {"User-Agent": "lodestar-sar-terrain-study/0.1 (research script)"}

# trailhead (lat, lon), published trailhead elevation (m) used as a check
SITES = {
    "yosemite_happy_isles": ("Yosemite: Happy Isles", 37.7326, -119.5578, 1230),
    "sequoia_lodgepole": ("Sequoia: Lodgepole", 36.6046, -118.7250, 2050),
    "rocky_bear_lake": ("Rocky Mountain: Bear Lake", 40.3127, -105.6456, 2880),
    "olympic_hurricane_ridge": ("Olympic: Hurricane Ridge", 47.9697, -123.4985, 1590),
    "rainier_paradise": ("Mount Rainier: Paradise", 46.7860, -121.7355, 1645),
    "smokies_newfound_gap": ("Great Smoky Mtns: Newfound Gap", 35.6110, -83.4251, 1538),
    "whites_pinkham_notch": ("White Mtns NH: Pinkham Notch", 44.2576, -71.2532, 620),
}


def to_local(lat, lon, lat0, lon0):
    """Equirectangular east/north metres around the trailhead (error < 0.1% over 15 km)."""
    R = 6371000.0
    x = np.radians(np.asarray(lon) - lon0) * R * math.cos(math.radians(lat0))
    y = np.radians(np.asarray(lat) - lat0) * R
    return x, y


def tile_xy(lat, lon, z):
    n = 2**z
    x = (lon + 180) / 360 * n
    y = (1 - math.asinh(math.tan(math.radians(lat))) / math.pi) / 2 * n
    return x, y


def fetch_dem(lat0, lon0):
    R = 6371000.0
    dlat = math.degrees(HALF / R) * 1.05
    dlon = math.degrees(HALF / (R * math.cos(math.radians(lat0)))) * 1.05
    x0, y0 = tile_xy(lat0 + dlat, lon0 - dlon, ZOOM)
    x1, y1 = tile_xy(lat0 - dlat, lon0 + dlon, ZOOM)
    tx = range(int(x0), int(x1) + 1)
    ty = range(int(y0), int(y1) + 1)
    mosaic = np.zeros((256 * len(ty), 256 * len(tx)), np.float32)
    for j, y in enumerate(ty):
        for i, x in enumerate(tx):
            url = f"https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{ZOOM}/{x}/{y}.png"
            r = requests.get(url, headers=UA, timeout=60)
            r.raise_for_status()
            a = np.asarray(Image.open(BytesIO(r.content)).convert("RGB"), np.float32)
            mosaic[j * 256:(j + 1) * 256, i * 256:(i + 1) * 256] = a[..., 0] * 256 + a[..., 1] + a[..., 2] / 256 - 32768
    # sample onto the local grid
    n = int(2 * HALF / CELL)
    c = (np.arange(n) + 0.5) * CELL - HALF
    X, Y = np.meshgrid(c, c[::-1])                       # row 0 = north edge
    lat = lat0 + np.degrees(Y / R)
    lon = lon0 + np.degrees(X / (R * math.cos(math.radians(lat0))))
    n2 = 2**ZOOM
    px = ((lon + 180) / 360 * n2 - tx.start) * 256 - 0.5
    py = ((1 - np.arcsinh(np.tan(np.radians(lat))) / math.pi) / 2 * n2 - ty.start) * 256 - 0.5
    return map_coordinates(mosaic, [py, px], order=1).astype(np.float32)


def fetch_osm(lat0, lon0):
    R = 6371000.0
    dlat = math.degrees(HALF / R)
    dlon = math.degrees(HALF / (R * math.cos(math.radians(lat0))))
    bb = f"{lat0 - dlat},{lon0 - dlon},{lat0 + dlat},{lon0 + dlon}"
    q = f"""[out:json][timeout:180];
(way["highway"~"^(path|footway|track|bridleway|steps|cycleway)$"]({bb});
 way["highway"~"^(motorway|trunk|primary|secondary|tertiary|unclassified|residential|service|road)$"]({bb});
 way["waterway"~"^(river|stream)$"]({bb}););
out geom;"""
    for attempt in range(4):
        r = requests.post("https://overpass-api.de/api/interpreter", data={"data": q}, headers=UA, timeout=240)
        if r.status_code == 200:
            return r.json()
        time.sleep(20 * (attempt + 1))
    r.raise_for_status()


def classify(tags):
    h, w = tags.get("highway"), tags.get("waterway")
    if w:
        return "drainage"
    if h in ("path", "footway", "track", "bridleway", "steps", "cycleway"):
        return "trail"
    return "road"


CANOPY_WCS = ("https://www.mrlc.gov/geoserver/wcs?SERVICE=WCS&VERSION=1.0.0&REQUEST=GetCoverage"
              "&COVERAGE=mrlc_NLCD-Tree-Canopy_conus_year_data:NLCD-Tree-Canopy_conus_year_data"
              "&CRS=EPSG:4326&FORMAT=GeoTIFF&TIME=2021-01-01T00:00:00.000Z")


def fetch_canopy(lat0, lon0):
    """Percent tree canopy on the local 30 m grid, from a lat/lon GeoTIFF of the same extent (+5%)."""
    R = 6371000.0
    dlat = math.degrees(HALF / R) * 1.05
    dlon = math.degrees(HALF / (R * math.cos(math.radians(lat0)))) * 1.05
    w = h = int(2 * HALF * 1.05 / CELL)
    url = f"{CANOPY_WCS}&BBOX={lon0 - dlon},{lat0 - dlat},{lon0 + dlon},{lat0 + dlat}&WIDTH={w}&HEIGHT={h}"
    r = requests.get(url, headers=UA, timeout=300)
    r.raise_for_status()
    if "tif" not in r.headers.get("content-type", ""):
        raise RuntimeError(r.text[:500])
    a = np.asarray(Image.open(BytesIO(r.content)), np.float32)
    nodata = a > 100
    a[nodata] = 0.0
    n = int(2 * HALF / CELL)
    c = (np.arange(n) + 0.5) * CELL - HALF
    X, Y = np.meshgrid(c, c[::-1])                       # row 0 = north edge
    lat = lat0 + np.degrees(Y / R)
    lon = lon0 + np.degrees(X / (R * math.cos(math.radians(lat0))))
    py = (lat0 + dlat - lat) / (2 * dlat) * h - 0.5
    px = (lon - (lon0 - dlon)) / (2 * dlon) * w - 0.5
    return (map_coordinates(a, [py, px], order=0) / 100.0).astype(np.float32), float(nodata.mean())


def fetch_all_canopy():
    for key, (name, lat0, lon0, _) in SITES.items():
        f = DATA / key / "canopy.npy"
        if f.exists():
            continue
        cc, nodata = fetch_canopy(lat0, lon0)
        np.save(f, cc)
        print(f"{name:34s} canopy mean {cc.mean():.0%}, median {np.median(cc):.0%}, "
              f">=50% canopy on {(cc >= 0.5).mean():.0%} of the map; no-data {nodata:.1%}", flush=True)
        time.sleep(2)


if __name__ == "__main__" and "--canopy" in __import__("sys").argv:
    fetch_all_canopy()
elif __name__ == "__main__":
    for key, (name, lat0, lon0, elev) in SITES.items():
        d = DATA / key
        d.mkdir(parents=True, exist_ok=True)
        if not (d / "dem.npy").exists():
            np.save(d / "dem.npy", fetch_dem(lat0, lon0))
        dem = np.load(d / "dem.npy")
        if not (d / "osm.json").exists():
            raw = fetch_osm(lat0, lon0)
            lines = []
            for el in raw.get("elements", []):
                if el.get("type") != "way" or "geometry" not in el:
                    continue
                la = [p["lat"] for p in el["geometry"]]
                lo = [p["lon"] for p in el["geometry"]]
                x, y = to_local(la, lo, lat0, lon0)
                lines.append(dict(kind=classify(el.get("tags", {})), name=el.get("tags", {}).get("name", ""),
                                  x=np.round(x, 1).tolist(), y=np.round(y, 1).tolist()))
            (d / "osm.json").write_text(json.dumps(lines), encoding="utf-8")
            time.sleep(5)
        lines = json.loads((d / "osm.json").read_text(encoding="utf-8"))
        n = dem.shape[0]
        th = dem[n // 2, n // 2]
        km = {k: sum(np.hypot(np.diff(l["x"]), np.diff(l["y"])).sum() for l in lines if l["kind"] == k) / 1000
              for k in ("trail", "road", "drainage")}
        meta = dict(name=name, lat=lat0, lon=lon0, trailhead_elev_published=elev, trailhead_elev_dem=float(th),
                    elev_min=float(dem.min()), elev_max=float(dem.max()), km=km)
        (d / "meta.json").write_text(json.dumps(meta, indent=1), encoding="utf-8")
        print(f"{name:34s} DEM at trailhead {th:6.0f} m (published {elev}), relief {dem.min():.0f}-{dem.max():.0f} m, "
              f"trails {km['trail']:.0f} km, roads {km['road']:.0f} km, streams {km['drainage']:.0f} km")
