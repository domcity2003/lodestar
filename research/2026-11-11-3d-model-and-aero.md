# Milestone 3: 3D model, the centre pod, and an aero cross-check

2026-11-11 (US Central).

**Code:**
- `scripts/vsp_model.py`: OpenVSP model, VSPAERO, parasite drag
- `scripts/pod_np_check.py`: AeroBuildup and Raymer estimates
- `scripts/pod_avl_check.py`: AVL with the pod
- `scripts/render_preview.py` + `scripts/blender_scene.py`: Blender renders

**Outputs:** `out/vsp/` (the `.vsp3` model, STL, polars, drag build-up, logs) and `out/render/`.

## Answer first
**There's now a full 3D model** in OpenVSP, the format NASA's own conceptual-design tool uses. It includes the swept MH45 wing, 3° washout, 0.25 m winglets, a centre pod sized around the 3.4 L kit bay, and the camera ball under the nose. Previews in `out/render/` (hero shots and an X-ray of the internal layout).

Three findings:
1. **The pod makes the aircraft less stable in pitch, so the balance point moves 15 mm forward**, to x = 0.422 m. Four methods estimate the effect at 2.5-15 mm; I designed to the most cautious trusted one. A welcome side effect: the battery packs move forward to x = 0.458 m, where the wing is thicker. **The battery-fit problem from milestone 2 is gone** (24-47 mm of depth for 21 mm cells).
2. **The 0.25 m winglets still work with the pod.** The pod weakens yaw stability by the same amount in two independent tools (Cnb −0.007). Even so, every lateral mode still passes, and with the pod the spiral mode becomes fully stable.
3. **Drag is now the main open question, and 3 h is no longer certain.** The pod and camera came in on budget (42 cm² of drag area against 40 cm² assumed). The wing is the uncertain part:
   - It depends on how much of the wing keeps laminar (smooth, low-friction) flow.
   - Endurance ranges from **2.6 h** (wing fully turbulent) to **3.1 h** (laminar runs as NeuralFoil predicts).
   - A smooth, moulded composite wing is the way to stay near the top of that range.

## The model
| Part | Shape |
|---|---|
| Wing | MH45 airfoil; root chord 0.591 m; break at 0.45 m (start of the elevons), chord 0.494 m; tip chord 0.266 m; 25° quarter-chord sweep; twist 0 / −0.9° / −3° |
| Winglets | 0.25 m tall, 35° leading-edge sweep, taper 0.6, NACA 0008 |
| Centre pod | 0.82 m long, from x = −0.17 to 0.65 m. Rounded-rectangle sections, 160 × 150 mm at full size. Hangs below the wing root |
| Camera gimbal | 100 mm ball under the nose, about 60% exposed below the pod |
| Pusher prop | 13" (visual only, in the renders) |

**Kit fit check:** a 220 × 130 × 120 mm box (3.4 L) centred on the new balance point at x = 0.422 m spans x = 0.312-0.532 m. The pod is full-size (160 × 150 mm) over that whole range except the last 5 mm, where it is 155 mm wide. It fits with room for walls.

## Finding 1: the pod moves the neutral point forward
A body sticking out ahead of the balance point acts like a small backwards tail: it amplifies any nose-up bump. So the neutral point moves forward.

| Method | Shift | Notes |
|---|---|---|
| Raymer handbook (fuselage K_f) | −2.5 mm (−0.5% MAC) | Empirical, includes the wing's downwash over the rear of the pod |
| AeroSandbox AeroBuildup | −5.0 mm (−1.1% MAC) | Semi-empirical: lifting line + slender body |
| **AVL slender body** | **−15.0 mm (−3.3% MAC)** | Same tool as milestone 2; slender-body theory tends to overstate this |
| VSPAERO panel method | −10.4 mm (−2.3% MAC) | **Not trusted here**, see below |

**Decision:** keep a 6% static margin against the most cautious trusted estimate (AVL). The balance point moves from 0.437 to **0.422 m**. If the smaller estimates are right, the true margin is about 9%, which milestone 2 showed costs about 1% of endurance. Cheap insurance.

**New layout:**
- kit bay centred at x = 0.422 m;
- battery packs at x = 0.458 m, spanning y = ±0.25-0.46 m (outboard, where the swept wing is thicker);
- one layer of 21700 cells: 5 along the chord × 3 along the span per side.

**Why VSPAERO isn't trusted for the pod case.** VSPAERO solves surfaces with thickness ("panel method"), unlike AVL's flat surfaces.
- **Wing only, it's fine:** its neutral point (0.453 m, or 0.462 m with the symmetry option) is close to AVL's 0.464 m.
- **With the pod, it breaks:**
  - the lift slope falls 25% (not physical for a 16 cm pod on a 3 m wing);
  - the surface-integrated induced drag comes out negative;
  - with the symmetry option on, lift was negative outright.
- **Why:** the solver cuts away the wing inside the pod, and the pod sheds no wake to carry that lift. The "body wakes" option made no difference.

Getting this right needs a cleaner pod-wing junction mesh or a real CFD code. It goes on the list, not into the decision.

## Finding 2: the winglets still hold with the pod
AVL with the pod (`out/vsp/pod_avl_check.csv`), 0.25 m winglets, balance point re-set to 6% margin:

| Case | Cnb no pod → pod | Dutch roll damping no pod → pod | Spiral | All criteria |
|---|---|---|---|---|
| Search, 15 m/s, kit on | 0.023 → 0.016 | 0.110 → 0.129 | doubles in 58 s → stable | pass |
| Top speed, 22 m/s, kit on | 0.021 → 0.014 | 0.093 → 0.125 | 78 s → stable | pass |
| Top speed, kit off | 0.021 → 0.014 | 0.101 → 0.136 | 97 s → stable | pass |
| Slow, 12.5 m/s, kit off | 0.024 → 0.017 | 0.119 → 0.135 | 58 s → stable | pass |

AeroBuildup gives the same Cnb drop (−0.0070), from a completely different method. Less yaw stiffness slows the Dutch roll down and lets it damp better. It also stops the spiral from tightening, so the pod helps rather than hurts here. Trim elevon barely changes (within 0.1°).

## Finding 3: drag and endurance
**OpenVSP parasite-drag build-up** (component wetted area × skin friction × form factor, 15 m/s at 2,000 m):

| Part | Wetted area | Reynolds | Drag area |
|---|---|---|---|
| Wing + winglets | 2.65 m² | 377,000 | 172 cm² (all turbulent) |
| Pod | 0.33 m² | 717,000 | 19 cm² |
| Gimbal ball | | | 23 cm² (corrected, see below) |

The tool's streamlined-body formula is meant for long, slender shapes. On a sphere it gives a form factor of 53 and 194 cm² of drag area, which is nonsense. I replaced it with sphere drag (Cd 0.47) on the 49 cm² that hangs below the pod.

**Pod + gimbal: 42 cm², against the 40 cm² the sizing assumed. On budget.**

**The wing is the uncertain part.** OpenVSP assumes fully turbulent flow (CD 0.0134). NeuralFoil, used in the sizing, predicts laminar flow over much of the chord at this Reynolds number (CD 0.0090 including the sizing's ×1.15 factor). Real wings land in between, depending on surface finish, seams and bugs.

| Wing drag assumption | CD0 total | Endurance, e = 0.80 | Endurance, e = 0.90 |
|---|---|---|---|
| NeuralFoil (laminar runs, as sizing) | 0.012 | 2.95 h | 3.10 h |
| Halfway | 0.014 | 2.77 h | 2.90 h |
| Fully turbulent (OpenVSP) | 0.017 | 2.61 h | 2.73 h |

All at 7.55 kg, at the best speed of at least 1.25 × stall (15 m/s in every case). e is span efficiency.

**Ways to win back margin if needed:**
- **Battery cells:** 240 Wh/kg cells instead of 200 Wh/kg give about +20% energy, the biggest lever from the milestone 1 sensitivity study.
- **Smooth moulded composite wing:** keeps the laminar runs.
- **Retractable gimbal:** stowed in transit, it saves about half its 23 cm².

## Charts and images
- `out/render/hero.png`, `hero_low.png`: outer shape
- `out/render/xray_top.png`, `xray_iso.png`: kit bay (orange), battery packs (blue), balance point (black)
- `out/vsp/full/rescue_wing.vsp3`: opens in OpenVSP's GUI (`aero-tools/OpenVSP-3.56.0-win64/vsp.exe`)

## Tooling notes
- **PyVista rendering** was blocked by Windows Application Control (its VTK DLL). I didn't try to bypass it; renders use the portable Blender 5.3 in Cycles instead.
- **OpenVSP is driven by generated `.vspscript` files** through `vspscript.exe`, so no Python API install is needed.

## Caveats
1. **The neutral-point shift ranges from 2.5 to 15 mm across methods.** I designed to the cautious end; a CFD run or a glide test would narrow it.
2. **The pod is modelled as round in AVL** (equal cross-section area) and as ellipses in AeroBuildup; the real pod is a rounded rectangle.
3. **The wing's drag depends on transition**, which none of these tools model reliably for a real surface finish. Flight test is the true answer.
4. **No propeller effects:** slipstream, thrust line, or the prop's effect on pitch trim and yaw.
5. **The renders are previews**, not the final CAD. Elevons, hatches and the spar aren't modelled yet; that's milestone 5.

## What feeds the next milestones
- **Layout rules for CAD:**
  - kit bay centred at x = 0.422 m (allowed 0 to +5 cm aft, never forward);
  - battery packs at x ≈ 0.458 m, y = ±0.25-0.46 m;
  - gimbal at the front of the pod; pusher motor at x ≈ 0.64 m.
- **Mission simulation (milestone 4):** use endurance 2.6-3.1 h as the range, and test search patterns in wind at both ends.
- **Cost model:** price a moulded composite wing against foam-and-glass. Surface finish now directly affects endurance.
