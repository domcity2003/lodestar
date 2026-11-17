"""Lodestar v1 frozen design (revision v1.1): the single parameter source.

Every number another engineer needs to build or check v1 is here, with the file that produced it.
`freeze_check.py` verifies that the analysis scripts and their outputs agree with these values,
so a later change that is not carried through everywhere fails loudly.

Units: SI (m, kg, s, N, Pa) unless the name says otherwise. Axes: x aft, y right, z up, root LE at x = 0.
"""

VERSION = "v1.1"                 # v1 release: v1.0 freeze (2026-11-14) + the Step 2 changes (research/2026-11-16-step2-results.md)
FROZEN = "2026-11-16"

GEOMETRY = dict(
    span=3.0, c_root=0.5911, c_tip=0.2660, sweep_qc_deg=25.0, washout_deg=3.0, dihedral_deg=0.0,
    airfoil="MH45", y_break=0.45, hinge_xc=0.75,
    winglet_h=0.25, winglet_taper=0.6, winglet_sweep_deg=35.0, winglet_airfoil="NACA0008",
    S_ref=1.28565, mac=0.4491,
    pod_x0=-0.17, pod_len=0.82, pod_z=-0.035,
    pieces=3,                        # centre section + two outer panels
)

STRUCTURE = dict(
    design="v1",                     # wing_trade.DESIGNS key (compact_eps20_light wing, CAD v4 layout)
    skin="1 ply +/-45 glass 80 g/m2, wet layup",
    core="EPS 20 kg/m3; XPS at servo bay (+/-0.05 m) and winglet root (y > 1.40 m)",
    skin_doubler="one extra +/-45 glass ply on the upper skin, y 0-0.45 m, dropped off over 0.45-0.50 m",
    spar="pultruded carbon caps 12 mm wide on Rohacell 51 web at 25% chord; rear web at 72% chord",
    joint_tube_od=0.020, joint_tube_id=0.016, joint_sleeve_od=0.024, joint_engagement=0.160,
    joint_allowance_kg=0.045,        # per break: local blocks, wraps, aft key, retention, adhesive
    wing_mass_kg=2.006,              # incl. joints and servos, excl. winglets (out/wing_trade/summary_v1.csv; v1.0 1.983)
    fos=1.5, n_pos=3.8, n_neg=-1.5, vc_eas=22.0, vd_eas=30.0,
    wrinkle_k=0.3,
    # flutter_v2 (research/2026-11-14-flutter-verification.md): requirements the hardware must meet
    joint_min_bending_k_nm_rad=1000.0, joint_target_bending_k_nm_rad=2000.0,
    elevon_hinge_k_nm_rad=136.0, elevon_hinge_k_min_nm_rad=20.0, elevon_free_play_max_deg=0.15,
    servo_failsafe="hold position on signal loss; non-back-drivable",
    # fea_wing (out/fea/margins.csv): details without which a naive build has a negative margin.
    # Masses fall inside the 45 g-per-break joint allowance or the 5% misc allowance.
    upper_skin_doubler_y=(0.0, 0.45),    # one extra +/-45 glass ply on the centre-section upper skin (FE shear peak)
    upper_skin_doubler_taper_m=0.05,     # ply drop-off outboard of the doubler: 1.5x skin over this length
    cap_ply_drop_factor=1.25,            # v1.1: FE skin stress 1.08-1.20x sizing at abrupt cap drops (sizing rule)
    belly_skid_crush_stroke_min_mm=70.0,   # v1.1: 4.5 g half-sine at the 2.0 m/s design sink needs 71 mm (6-DOF note)
    belly_skid_material="EPP 20 g/L, tapered contact face",
    belly_skid_mean_crush_force_n=(250.0, 360.0),   # v1.1: spec by force too; a flat 40 mm EPP block gave 9-33 g
    landing_design_sink_mps=2.0,         # v1.1: 6-DOF turbulent landings P95 1.58, max 1.81 m/s (light turbulence)
    tip_wear_pads="elevon tip trailing edge + winglet root, replaceable glass/Kevlar pads (tip strike above 0.9 deg bank)",
    construction_details=(
        "sleeve ends seated on two 1.5 mm birch-ply/G10 ribs each (not bearing on EPS)",
        "sleeves bonded through a glass wrap onto the Rohacell spar web and caps (not into foam alone)",
        "joiner tube with hoop plies at the sleeve mouths (ovalization margin +0.04 at 150 MPa transverse)",
        "wing-pod saddle: 2 x 60 x 40 mm G10 plates over the EPS at the pod pads",
        "battery bay: keep a structural lower lid, M3 nylon screws in G10 inserts at 30 mm pitch",
        "winglet joint: 2 x 25 mm glass overlap tapes (foam butt joint alone fails)",
        "spar cap ply drops staggered over >= 25 mm each (v1.1; FE skin peak at abrupt drops)",
    ),
)

MASS = dict(
    # out/wing_trade/aircraft_mass_balance_v1.csv (spanwise wing model + CAD v4 positions)
    # v1.1 (Step 2 changes). v1.0 freeze: 8.315 / 6.691 / 8.281 / 6.657 kg, CG 0.4134 / 0.4113, SM 7.07 / 7.44 %
    sicore_loaded_kg=8.755, sicore_released_kg=7.131,
    p50b_loaded_kg=8.721, p50b_released_kg=7.097,
    x_cg_sicore_loaded=0.4126, x_cg_sicore_released=0.4104,
    static_margin_loaded_pct=7.28, static_margin_released_pct=7.66,   # level trim, AVL with pod
    rescue_kit_kg=1.50, kit_chute_kg=0.124, kit_container_kg=0.08,   # container + release stay on board
    mtow_limit_kg=10.0,
)

LAYOUT = dict(
    x_cg_design=0.422,               # kit bay centred here
    x_batt=0.447, y_batt=0.355,      # v1.1: 5 mm fwd of v3 (v1.0: 0.412 / 0.305); nose parachute now trims
    cells_per_side="5 chordwise x 3 spanwise 21700, axis spanwise",
)

PROPULSION = dict(
    motor="T-Motor AT3520 class, 50 A / 180 s rating", prop="15 x 10 in folding", esc="60 A with RPM telemetry",
    esc_current_limit_a=45.0,        # worst requirement needs 40.6 A (research/2026-11-14-propulsion-battery-margins.md)
    pack_default="Amprius SiCore (SA112) 6.3 Ah 6S5P", pack_alt="Molicel P50B 6S5P",
    sicore_c_rating_cont=2.0,        # SA112 datasheet (repo previously said 3C)
    pack_min_temp_c=15.0,            # held by 20 mm EPS around each pack + 5 W thermostat pad
    heater_w=5.0, pack_insulation_mm=20.0,
    heater_cutout_c=50.0,            # v1.1 (RC3): stuck-on heater takes cells to 65 C on a +40 C day
    power_lead="8 AWG pack to ESC (10 AWG only if routed off the foam: 83 C in foam at full climb, +40 C)",
)

OPERATIONS = dict(
    design_alt_m=2000.0, max_alt_m=4400.0, search_speed_tas=(18.0, 20.0), stall_margin=1.25,
    max_bank_deg=30.0, chute_v_land=7.0, release_agl_m=40.0, launch="bungee or catapult",
    cl_max_3d=0.80,
    launch_release_min_tas=16.0,     # required above 3,000 m density altitude; packs pre-warmed before launch
    no_full_power_climb_below_soc=0.25,   # enforced by the bingo rule below
    no_full_power_climb_below_soc_above_3000m_da=0.35,   # v1.1: +0.42 kg left 2.82 m/s / 13.9% at 4,400 m, -10 C
                                     # with 25%; 35% gives 3.09 m/s / 15.3% (5 W heater already at its 15 C limit)
    bingo_rule="head home when energy less 10% landing reserve <= 1.2 x E_return(distance, wind + 3 m/s)",
    landing_reserve_frac=0.10,
    turbulence_min_tas=18.0,         # moderate turbulence: 15 m/s leaves no gust stall margin (controls note)
    drop_runin_tas=dict(calm=18.0, light=18.0, moderate=20.0),   # v1.1: 14 m/s stalls 49/50 in light turbulence
    launch_full_power_above_da_m=3000.0,   # v1.1: motor at full power at release (no throttle delay) above this DA
    launch_release_alt_tas=16.5,     # v1.1: or a 16.5 m/s release with the normal throttle delay
    flare_pitch_max_deg=4.5,         # v1.1: limits tail strike on the belly landing
    search_tas_above_3200m_da=20.0,  # v1.1: 30 deg turns at 18 m/s fall below 1.25 Vs above ~3,200 m DA
    max_crosswind_launch_deg=30.0,   # no rudder: launch within 30 deg of the wind
)

CONTROLS = dict(
    # research/2026-11-14-flight-control-physical.md, scripts/controls_sizing.py
    cg_fwd_m=0.365, cg_aft_m=0.422,  # aft = 5% static margin on the most-forward neutral point
    cg_build_tol_m=0.004,
    elevon_travel_deg=(-20.0, 20.0), linkage_ratio=2.0,
    servo_min_torque_nm=0.9, servo_max_s_per_60deg=0.12, servo_max_thickness_mm=13.0,
    servo_part="KST A12-S HV",       # v1.1: 1.35 N m at 8.4 V, 0.08 s/60 deg, 12 mm (X12-508 is 0.61 N m: fails)
    servo_torque_nm=1.35, servo_s_per_60deg=0.08, servo_thickness_mm=12.0,
    hinge_moment_design_nm=0.89, hinge_stiffness_nm_rad=136.0, hinge_free_play_deg=0.15,
    pitot_ahead_of_nose_m=0.230, pitot_tilt_deg=-6.0,
    power_leads="twisted pairs (single leads put 21-36 uT at the compass)",
    roll_60deg_s=0.52,               # worst case, full travel; MIL-F-8785C Class I Level 1 limit 1.3 s
)

# Mass added by the Step 1 fixes. cad_v4.py adds these rows to out/cad_v4/mass_cg.csv, so the
# wing_trade mass balance, loads and performance carry them.
ADDED_MASS = dict(
    pack_thermal_kit_kg=0.10,        # 20 mm EPS around both packs (50-100 g, upper end) + 5 W heater pads
    elevon_balance_kg_each=0.060,    # flutter_v2: full static balance of the elevon (2.23 g*m)
    elevon_balance_y=0.87, elevon_balance_arm=0.040,   # at the servo station, 40 mm ahead of the hinge
)

# ---------------------------------------------------------------- v1.1 (Step 2: tests and improvements)
# Mission-sim winners on the tuning seed (out/step2_trades, both detection calibrations):
#   twin splayed camera +1.9 to +4.8 pp help-in-1h/3h (95% CIs exclude zero in all 8 cells);
#   40 deg search-turn bank on top of the twin +0.4 to +1.3 pp (positive in 8/8, CI excludes zero in 4/8).
#   16 m/s search: no significant change; 20 m/s: worse. Search speed stays 18 m/s.
PAYLOAD = dict(
    budget_v1_kg=0.60,               # v1 carried the sizing budget for a dual thermal + RGB gimbal
    gimbal_part="SIYI ZT6 class (thermal 640 + RGB), 0.197 kg",
    nadir_thermal="FLIR Boson 640, 50 deg HFOV, 0.07 kg with lens and mount",
    twin_second_core="second Boson 640 core with 8.7 mm lens + splayed mount, 0.09 kg",
    part_kg=0.197 + 0.07,            # single nadir camera + gimbal (v1 hardware at part mass)
    twin_extra_kg=0.09,
)
CAMERA_V11 = dict(
    layout="two Boson 640 cores splayed +-22.5 deg across track",
    hfov_deg=95.0, pixel_tan=0.46630765815,   # tan(25 deg): each core keeps the 50 deg lens's pixel size
    track_spacing_m=135.7,           # no-blur sweep-width ratio x 67.5 m (step2_trades.twin_camera)
    search_bank_deg=40.0,            # autopilot setting; given to every aircraft in the comparison runs
    load_factor_in_turn=1.305,       # 1/cos 40 deg, inside n_pos 3.8; stall-margin speed rises x 1.142
)
# Each row: (mass kg, x m, z m, basis). Added to the CAD v4 mass table and to the power model's mass.
V11_MASS = {
    "payload at part mass (was 0.60 kg budget)": (PAYLOAD["part_kg"] - PAYLOAD["budget_v1_kg"], 0.07, -0.113,
                                                  "ZT6 0.197 + Boson 0.07 kg vs the 0.60 kg sizing budget"),
    "twin camera second core + splayed mount": (PAYLOAD["twin_extra_kg"], 0.07, -0.113, "step2_trades winner"),
    "aircraft parachute (RC1, ASTM F3322) + independent trigger": (0.40, 0.06, -0.015,
        "Fruity Chutes HUAV class ~0.32 kg for <= 9.3 kg UAV + launcher/trigger; canister in the pod nose"),
    "backup avionics supply + locking connectors (RC2)": (0.07, 0.22, -0.03, "2S backup cell ~50 g + clips"),
    "BMS, heater 50 C cut-out, cell liner (RC3)": (0.05, LAYOUT["x_batt"], 0.008, "at the packs"),
    "ADS-B In receiver + anti-collision strobes (RC8)": (0.03, 0.30, -0.02, "receiver in pod; strobes on winglets"),
    "second GNSS + antenna (RC10)": (0.04, 0.14, 0.05, "on the mast ahead of the first"),
    "8 AWG pack leads (increment over 10 AWG)": (0.045, 0.49, -0.01, "~28 g/m x 1.6 m"),
    "70 mm EPP belly skid (increment over 4 mm pad)": (0.025, 0.27, -0.13, "0.30 x 0.06 x 0.07 m tapered, 20 g/L"),
}
CDA_INCREMENT_V11_M2 = 0.0004        # skid (~0.0003) + strobes/antenna (~0.0001); v1 CDA_BODY 0.00423
# Design decisions taken in Step 2 (owner: Dominic).
DECISIONS = {
    "2026-11-16 performance before cost": (
        "Keep the twin Boson camera and the full safety fit (RC1-RC10). R13 (parts <= $8k) is deferred to a later "
        "cost-down pass, not used as a gate: v1.1 parts ~$16.4k (out/cost_v1.1). The v1 compliance note's $7,760 "
        "omitted the v4 Boson nadir camera, so v1 already exceeded R13 (~$12.4k)."),
}
