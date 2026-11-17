# Lodestar preliminary wing structure

2026-11-12, US Central.

The [removable wing joint trade](2026-11-12-wing-redesign.md) reruns these preliminary results with explicit joint mass, roll inertia and damping, and both kit states.

## Results

The lightest modeled wing weighs **1.95 kg**, versus the **1.29 kg** structure budget. The added 0.66 kg needs to flow through the aircraft mass, balance, and mission models. The section estimate includes skins, core, spars, elevons and servos. It excludes the winglets and does not separately size or weigh joiner sleeves, reinforcements, fasteners or adhesive load paths. It uses representative composite properties rather than measured properties from a controlled layup.

| Layup | Wing mass | Max limit tip deflection | Joiner stress, symmetric ultimate | Joiner stress, fixed-aileron sensitivity | Sea-level clamped-wing flutter |
|---|---:|---:|---:|---:|---:|
| One ply glass | 1.95 kg | 32 mm | 282 MPa | 542 MPa | 105.5 m/s TAS |
| Two plies glass | 2.36 kg | 25 mm | 251 MPa | 530 MPa | 139.3 m/s TAS |
| One ply carbon | 1.97 kg | 26 mm | 280 MPa | 542 MPa | 163.6 m/s TAS |

The assumed 500 MPa joiner flexural allowable clears the symmetric nominal tube-stress calculation and is exceeded in the fixed-aileron sensitivity for every layup. The fixed-aileron case uses a stationary AVL flow solution with 15° differential deflection. Roll acceleration, aerodynamic roll damping, section stall and real actuator limits can change the load. Resolve the asymmetric case before selecting a joiner. The uncapped positive gust sensitivity gives 330 MPa for one-ply glass, versus 282 MPa in the capped symmetric envelope.

At 2,000 m the existing power model, updated only for wing mass, estimates 8.10 kg takeoff mass and **3.14 h** at 20 m/s with SiCore, down from 3.27 h. P50B falls from 2.90 h to **2.75 h** at 18 m/s. Both figures include the existing battery and reserve allowances. Modeled stall speeds rise to **12.40 m/s TAS** and **12.37 m/s TAS**, respectively (11.23 and 11.20 m/s EAS). The original 12 m/s requirement should specify the airspeed convention and, if applicable, the design altitude. These results precede the joiner and reinforcement changes.

Replacing the wing entry in the v3 P50B CAD mass table, with all other entries fixed, shifts the loaded centre of gravity **4.7 mm aft**. Holding the level-trim AVL neutral point fixed gives a static margin of **4.0% of mean aerodynamic chord**, down from 5.0% in the same sensitivity. The CAD mass substitution totals 8.03 kg. The 8.06 kg P50B power-model case applies its existing miscellaneous mass fraction to the heavier wing.

## Verification and scope

- The original flutter run stopped at an indefinite mass matrix. Consistently integrating the bending–torsion mass terms repaired it. At sea-level density the Goland benchmark gives **137.25 m/s and 70.08 rad/s** versus **137.1 m/s and 11.0 Hz (69.12 rad/s)** for the published two-dimensional unsteady-aero comparison. Reducing the mesh from 30 to 15 elements changes the benchmark speed by about 0.02 m/s. For the one-ply glass Lodestar case, 15, 30 and 60 elements with 4, 6 and 8 retained modes respectively give 103.32, 105.47 and 105.82 m/s at sea level. [Goland comparison](https://www.mdpi.com/2076-3417/16/11/5687)
- The corrected shear-centre calculation reproduces the midpoint of a uniform rectangular closed box. The deflection integrator reproduces the analytical tip-loaded constant-stiffness cantilever within 0.001%. Signed AVL trims are checked for lift and pitch residuals before they are used. Each symmetric case contributes to the station-by-station load envelope. [AVL primer](https://web.mit.edu/drela/Public/web/avl/AVL_User_Primer.pdf)
- Static loads use sea-level density, the nominal 7.40 kg starting design mass, and project-selected gust and manoeuvre cases informed by CS-VLA. The stall caps and dynamic overshoot are estimates. Uncapped gusts are retained as sensitivities. The loads have yet to be recomputed for the heavier mass, alternative payload states and critical altitudes. [EASA CS-VLA §§333–349](https://www.easa.europa.eu/sites/default/files/dfu/Easy%20Access%20Rules%20CS-VLA%20%28Amendment%201%29.pdf)
- The flutter calculation uses an idealized straight beam and a fixed root. The project screening threshold is 36 m/s EAS, equivalent to 36.0 m/s TAS at sea level and 45.0 m/s TAS at 4,400 m under the modeled densities. Free-flight pitch coupling, sweep, joint compliance, control-surface free play and dynamics, divergence, and reversal remain open. The listed speeds describe the modeled bending–torsion modes. [NASA body-freedom flutter flight tests](https://ntrs.nasa.gov/citations/20220012337)

## Reproduce

From the repository root, run `uv run python -X utf8 scripts/structures.py` with the shared AVL executable installed under `../aero-tools/avl/`. The run writes `out/structures/` with the verification log, signed load cases, section tables, flutter curves and mass sensitivity. In the load tables, `V`, `Mb` and `T` are limit-level section shear, bending and torque in N, N·m and N·m. The section tables contain ultimate envelopes. The performance sensitivity varies wing mass while holding aerodynamic, propulsion and battery assumptions fixed.

## Next engineering checks

1. Close the wing and aircraft mass budget with a weighed bill of materials and explicit joiner, sleeve and attachment design. Update the balance model, then regenerate the load cases for the revised aircraft.
2. Calculate the asymmetric loads with actual elevon limits, rate damping and angular inertia. Verify the joint load path for bending, torsion and shear, including local bearing, bond and failure modes. Plan coupon and joint tests before relying on material allowables. [Evonik Rohacell data](https://products.evonik.com/assets/35/34/ROHACELL_IG_F_2022_April_EN_243534.pdf), [FAA composite structure guidance](https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC_20-107B_with_change_1.pdf)
3. Correlate the assembled wing stiffness and modes with ground tests before assessing aircraft-level flutter. Use the updated mass and validated operating envelope to rerun mission performance.
