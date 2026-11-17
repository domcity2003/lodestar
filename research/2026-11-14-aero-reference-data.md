# Published aerodynamic data for small flying wings

2026-11-14

## Plain summary

- The best citable full dataset for a small swept flying wing is the Skywalker X8 (2.1 m span, 3.4 kg). Its coefficients come from flight system identification at 18 m/s, close to Lodestar's 18-20 m/s cruise ([Løw-Hansen et al. 2025](https://elib.dlr.de/212985/1/s13272-025-00816-3.pdf)). The X8 is smaller and lighter than Lodestar, so use it for order-of-magnitude checks (CL_alpha, Cm_alpha, static margin, CD0), not as a target.
- I could not read the original Gryte et al. 2018 wind-tunnel paper (DOI [10.1109/ICUAS.2018.8453370](https://doi.org/10.1109/icuas.2018.8453370)); the PDF was not retrievable by my tools. Pull its tables by hand before quoting wind-tunnel values.
- Not found: one published wind-tunnel dataset for a swept flying wing at Re 150k-500k reporting CLmax, Oswald e and neutral point together. Treat those as open items.
- Washout guidance exists qualitatively (Hepperle); see the last section.

## Skywalker X8 flight-identified model (stability frame, 18 m/s)

Source: [Løw-Hansen et al. 2025, CEAS Aeronaut. J.](https://elib.dlr.de/212985/1/s13272-025-00816-3.pdf), Tables 1 and 8. Model form: CL = CL0 + CLα·α + CLq·q + CLδe·δe; CD = CD0 + k1·CL + k2·CL² (plus thrust and pitch-rate terms).

| Quantity | Value | Conditions |
|---|---|---|
| Span / area / mean chord / mass | 2.10 m / 0.75 m² / 0.36 m / 3.364 kg | Table 1 |
| Aspect ratio | about 5.9 | my arithmetic, b²/S |
| CL0 | -0.077 | flight ID, 18 m/s |
| CL_alpha | 2.573 per rad (0.0449 per deg) | same |
| CL_delta_e (elevon) | 1.369 per rad | same |
| CD0 | 0.058 | includes fuselage, prop, winglets |
| k1, k2 | -0.034, 0.225 | same |
| Cm0 | 0.027 | same |
| Cm_alpha | -0.274 per rad | same |
| Cm_delta_e | -0.276 per rad | same |
| Cmq | -1.608 | same |

Derived by me (check before use):
- Static margin is about -Cm_alpha / CL_alpha = 0.274 / 2.573, roughly 10.6 % of mean chord ahead of the neutral point at the identified CG.
- Implied Oswald e from k2 is 1 / (π · AR · k2), about 0.24. That is far below a clean-wing value because k2 absorbs fuselage and propwash effects, so do not compare it to an AVL span efficiency.
- The paper says the model has no stall description and loses accuracy at low or negative angles of attack, so it is valid only near trim. The pages I read give no sweep value.

Earlier X8 work: [Gryte et al. 2018](https://doi.org/10.1109/icuas.2018.8453370) (wind-tunnel static coefficients) and [Reinhardt, Gryte, Johansen](https://hdl.handle.net/11250/3052344) (flight-test velocity-based parameters), as described in the 2025 paper's literature review.

## Other data sources worth pulling

| Need | Source | Conditions | Status |
|---|---|---|---|
| Low-Re finite-wing CLmax, lift slope, aerodynamic centre, Oswald e trends | [Ananda, Sukumar, Selig 2015](https://m-selig.web.engr.illinois.edu/pubs/AnandaSukumarSelig-2015-AeroSciTech-LowReWings.pdf) | Re 60k-160k, unswept | Numbers not extracted; Re below Lodestar's range |
| Airfoil polars (MH-series and others) | [Airfoils at Low Speeds](https://m-selig.ae.illinois.edu:443/uiuc_lsat/Airfoils-at-Low-Speeds.pdf); [UIUC Vol. 1](https://m-selig.web.engr.illinois.edu/uiuc_lsat/AirfoilsVolume1.pdf) | Wind tunnel, Re 60k and up | Check whether an MH45 polar near Re 200k exists |
| Airfoil Cm0, zero-lift angle | [aerodesign.de table](https://www.aerodesign.de/english/profile/profile_s.htm) | MH45: Cm0 about +0.007, alpha_L0 about -0.27 deg | Secondary source; verify against [Hepperle](https://www.mh-aerotools.de/airfoils/nf_1.htm) |
| Forward-swept low-Re stall behaviour | [Randle and Miller, SAE 931370](https://www.sae.org/publications/technical-papers/content/931370/?PC=DL2BUY) | Re 0.25-0.75 million | Outboard flow stays attached past stall; forward sweep, so not directly applicable |
| VLM codes versus CFD on a UAV | [TU Delft thesis](https://repository.tudelft.nl/file/File_7eae8427-a10a-4341-b2ba-921b75f0d5bb) | AVL, XFLR5, Tornado, VSPAERO vs STAR-CCM+ | Closest multi-code comparison found; not read in full |

## Stall and washout guidance

- Swept, tapered wings load the tips more heavily, so the tips stall first unless twist is added ([Hepperle, flying-wing airfoils](https://mh-aerotools.de/airfoils/foil_flyingwings.htm)).
- Hepperle advises setting twist for one trimmed CL between best glide and best climb, and fixing planform, airfoil and twist together early because twist is hard to change later ([same page](https://mh-aerotools.de/airfoils/foil_flyingwings.htm)).
- Washout also supplies stability in a tailless wing, and Lippisch used it to reduce tip-stall risk ([Hepperle](https://www.mh-aerotools.de/airfoils/nf_1.htm)).
- Practical check for Lodestar: in AVL near stall, confirm the inboard section reaches its CLmax before the outboard sections. I found no verifiable numeric washout angle.

## VSPAERO and OpenVSP availability

- Windows wheels: OpenVSP has a Python API, but I found no official, current pip-on-Windows guide. Forum threads say Windows binaries are built for specific Python versions (3.50.1 reported for 3.11 and 3.13) and that installing from the `python/` folder with `requirements-dev.txt` places VSPAERO correctly ([forum thread](https://groups.google.com/g/openvsp/c/ZnoeajplqCQ)). One user still could not import `degen_geom` on Windows 11, with no resolution in the thread.
- A third-party project says `pip install openvsp` or prebuilt wheels from openvsp.org work ([AeroDemonstrator notes](https://agodemar.github.io/AeroDemonstrator/_sources/installation.rst.txt)); unverified until tested. Use 3.48.2 or newer ([forum](https://groups.google.com/g/openvsp/c/KoEx1iRnkJ0)).
- Headless: the official build docs describe a headless batch build of the API without graphics libraries ([OpenVSP repository](https://github.com/OpenVSP/OpenVSP)). The usual route is to build geometry with the Python API and call the VSPAERO solver from a script; I did not verify exact call names.
- Fallback: run `vspaero` from the install by subprocess on a saved `.vsp3` plus setup file.

## VSPAERO versus AVL agreement

- No formal paper compares the two for a flying wing. Forum reports: lift far above AVL on a disjointed-section model, blamed on poor spanwise paneling ([thread](https://groups.google.com/g/openvsp/c/24pvAUR2ZGk)); divergence from AVL above about 10 deg alpha on a straight NACA 0010 wing ([thread](https://groups.google.com/g/openvsp/c/9UP6htxR6YI)).
- VSPAERO's neutral point comes from the Cm-versus-CL slope of two cases, and its maintainer suggests cross-checking with AVL or Tornado ([thread](https://groups.google.com/g/openvsp/c/ggniwGJAuSU)). Neutral-point results also depend on panel count ([Mason slides](https://archive.aoe.vt.edu/mason/Mason_f/SubsonicWingsPres.pdf)), so run a panel-convergence study first.
- Suggested acceptance target (my proposal, not from a source): neutral point from AVL, AeroSandbox and VSPAERO within a few percent of MAC at matched paneling, and CL_alpha within about 5 percent.
