# Milestone 6: cost model, and what the money buys

2026-11-12 (US Central). Analysis by Claude Opus 5.5 (claude-opus-5-5).

**Code:**
- `scripts/cost_model.py`: parts list, unit cost, cost per hour, alternatives
- `scripts/cost_trades.py`: cost vs mission-sim score

**Outputs:** `out/cost/`
- `parts_list.csv`, `unit_cost.csv`, `cost_per_hour.csv`, `vs_alternatives.csv`, `sensitivity.csv`, `trades.csv`
- charts: `unit_cost_breakdown.png`, `cost_per_sortie.png`, `trades.png`

## Answer first
**One Rescue Wing costs about $7,300 to build. A 3-hour search costs about $80 in aircraft costs.** For comparison:
- **Sheriff's helicopter:** $3,600-12,000 for the same 3 hours.
- **Civil Air Patrol light aircraft:** $280-510.
- **Rescue Wing's parts:** about $5,200, well inside the $8,000 parts requirement.

**The cost model changes one design call from v3.** The best value is the SiCore cells with the cheaper foam-core wing, searching at 20 m/s:
- help reaches the person within 3 h in **50%** of lost-hiker missions, up from 40% for the cheapest build;
- that costs about $430 more per aircraft, or **$45 per percentage point**.

Moulded skins (v3's second change) are only worth it in one case: P50B cells and a fleet of about 25 or more aircraft. With SiCore cells they add nothing, because the aircraft already has energy to spare inside the 3-hour window.

| Recommended build | Unit cost | Help within 3 h (lost hiker) |
|---|---|---|
| **SiCore cells, foam-core wing, search at 20 m/s** | **$7,760** | **50%** |
| Fallback if SiCore cells can't be bought: P50B, foam-core, 18 m/s | $7,320 | 40% |
| Fallback for a fleet of 25+: P50B, moulded, 18 m/s | $7,940 at 10 built, $7,140 at 30 | 44% |
| Best fielded drone in the same missions | $21,000-60,000 | 17% |

## 1. Parts list (one aircraft, `parts_list.csv`)
Prices are US retail listings from 2026-11-12, mid case; the range is in brackets.

| Group | Main items | USD |
|---|---|---|
| Payload | SIYI ZT6 gimbal: 640×512 thermal + 4K, 3-axis | 2,940 (2,575-3,700) |
| Avionics | Pixhawk 6X + M10 GPS ($550); SIYI HM30 video, telemetry and RC link ($380); 2 KST X10 Pro servos; airspeed sensor; release servo; wiring | 1,210 (906-1,580) |
| Power | 30 Molicel P50B cells at $8.25 (200+ price tier); pack build | 308 (265-450) |
| Propulsion | T-Motor AT3520-550 ($80); Aeronaut CAM 15x10 blades ($25); hub and spinner; 60 A ESC | 230 (187-327) |
| Kit system | 1.5 m cross parachute; container and door hardware | 100 (60-150) |
| **Parts subtotal** | | **4,788 (3,993-6,207)** |
| Airframe, foam-core | materials $450 + 45 h labour at $45/h + $600 tooling ÷ fleet | 2,535 at 10 built |
| Airframe, moulded | materials $600 + 30 h labour + $12,000 moulds ÷ fleet | 3,150 at 10 built |

**The gimbal is 40% of the cost.** The airframe is the next-biggest item, and most of it is labour. A volunteer build saves about $2,000 per aircraft.

**Ground equipment is shared per team:** a laptop ($700, many teams already own one), a bungee launcher ($250) and a charger ($200).

**The kit is consumable**, about $85 per drop:
- FRS/GMRS radio, 0.5 L water, foil blanket, light, first-aid basics.
- Swapping the radio for a Garmin inReach Mini 2 satellite messenger makes it about $350. The person can then text rescuers from anywhere, not just within radio range.

## 2. Unit cost by build and fleet size (`unit_cost.csv`, paid labour)
| Aircraft built | Foam-core | Moulded |
|---|---|---|
| 1 | $7,860 | $18,740 |
| 3 | $7,460 | $10,740 |
| 10 | $7,320 | $7,940 |
| 30 | $7,280 | $7,140 |
| 100 | $7,270 | $6,860 |

The moulds ($12,000, a rough estimate) break even at about 25 aircraft. Beyond that, moulded is cheaper per aircraft and gives smoother skins.

## 3. Cost per flight hour (`cost_per_hour.csv`)
**Assumptions:**
- 60 sorties a year of 3 h each (call-outs plus training);
- a 400 h airframe life;
- the P50B pack retired after 600 cycles;
- repairs at 10% of the aircraft's cost a year;
- 10 kits dropped a year.

| Per flight hour | Foam-core, 10 built |
|---|---|
| Aircraft wear (depreciation) | $17.50 |
| Battery wear | $0.20 |
| Repairs and spares | $4.10 |
| Survival kits dropped | $4.60 (FRS radio) / $19.30 (inReach) |
| **Total, volunteer crew** | **$26** ($41 with inReach kits) |
| Total with 2 paid crew at $50/h | $126 |

Battery wear is almost free: about $310 per pack over 600 flights.

## 4. Against today's options (`vs_alternatives.csv`)
| Option | Per hour | 3-h search | Help within 3 h (lost hiker) | Cost per mission where help arrives in time |
|---|---|---|---|---|
| **Rescue Wing v3** | **$26** | **$79** | **40-50%** | **$165-200** |
| Quantum Trinity Pro, $21,260 (same ownership model) | $65 | $195 | 15% | about $1,300 |
| senseFly eBee VISION, $59,717 (same ownership model) | $183 | $547 | 12% (eBee X proxy) | about $4,700 |
| Civil Air Patrol Cessna 182, dry rate | $93 | $279 | not modelled | |
| Civil Air Patrol, national blended rate | $170 | $510 | not modelled | |
| Sheriff's helicopter | $1,200-4,050 | $3,600-12,150 | not modelled | |

**How to read it:**
- **Helicopters and crewed aircraft do things Rescue Wing can't:** they fly fast, carry trained spotters and, for a helicopter, lift the person out. The fair comparison is the **search hours** they burn before a find. Each hour Rescue Wing flies instead costs about 1-2% of a helicopter hour.
- **DeltaQuad doesn't publish a price**, so the Evo isn't in this table. In the mission sim it scores 17%, the same as the others, because none of them can drop a kit.

## 5. Cost vs design: what each upgrade buys (`trades.csv`, `trades.png`)
**Setup:** same 2,000 missions as milestones 4 and v3. The reference build is P50B, foam-core wing, 18 m/s. Costs assume 10 aircraft built.

| Option | Unit cost | Extra cost | Help within 3 h, lost hiker | Gain | USD per point |
|---|---|---|---|---|---|
| P50B, foam-core, 18 m/s (reference) | $7,320 | | 40.2% | | |
| P50B, moulded, 18 m/s | $7,940 | +$615 | 44.2% | +4.0 | $154 |
| SiCore, foam-core, 18 m/s | $7,760 | +$434 | 45.2% | +5.0 | $88 |
| **SiCore, foam-core, 20 m/s** | **$7,760** | **+$434** | **50.0%** | **+9.7** | **$45** |
| SiCore, moulded, 20 m/s | $8,370 | +$1,049 | 50.2% | +10.0 | $105 |

**Wide-area scenario (36 km², 10 km out):**
- SiCore, foam-core, 20 m/s: 20.7% against 16.8% for the reference.
- Every fielded drone: 0%.

**Why moulded skins stop mattering with SiCore cells:** in a 3-hour window, the SiCore aircraft already has more energy than it can use. Extra efficiency just means more battery left when it lands.

**That updates the v3 design:**
1. **Cells:** SiCore if it can be bought, otherwise P50B.
2. **Wing:** foam-core, unless a fleet of 25 or more aircraft is planned.
3. **Search speed:** 20 m/s with SiCore, 18 m/s with P50B. The motion-blur caveat from v3 applies at 20 m/s.

## 6. Sensitivity (`sensitivity.csv`, foam-core, 10 built)
| Change | Unit cost | Change |
|---|---|---|
| Mid case | $7,322 | |
| All parts at their low price | $5,958 | −$1,364 |
| All parts at their high price | $9,607 | +$2,284 |
| Gimbal at $3,700 (Irlock) | $8,082 | +$760 |
| Volunteer build (no labour cost) | $5,298 | −$2,025 |
| SiCore cells at $10 / $23 / $35 each | $7,375 / $7,756 / $8,125 | +$53 / +$434 / +$803 |

Even at the top of the SiCore range ($35 a cell), the SiCore build is $78 per point. That's still the best buy.

## Caveats
1. **Estimates without a published price:**
   - **Composite moulds:** anchored to one sailplane maker's ~€6,000 for CNC work and moulds, doubled for 3 wing pieces plus a pod.
   - **Build hours:** my estimate.
   - **SiCore cells:** no public price. Amprius sells them as evaluation samples, so supply is a real risk.
2. **Ownership assumptions:** airframe life, sorties a year and repair rates are assumptions. They move the hourly cost but not the ranking, because every aircraft in the table uses the same ownership model.
3. **Helicopter and Civil Air Patrol rates** are published figures from individual agencies (Sonoma County, LAPD audit, CAP), not a national survey.
4. **Not included:** certification, insurance, FAA waivers (beyond-visual-line-of-sight flight needs one for these missions), training, or the ground team.

## Sources
- SIYI ZT6: Druav $2,940; Irlock $3,699.90; Alibaba $2,575 (10+)
- Molicel P50B: 18650 Battery Store tiered pricing ($8.25 at 200+)
- Pixhawk 6X + GPS: Fruugo $549 (RRP $749)
- SIYI HM30: SIYI store $284-382
- T-Motor AT3520-550: Altitude Hobbies $79.99
- KST X10 Pro: $54.99-65
- Aeronaut CAM 15x10: Buchmann CHF 25.65
- Garmin inReach Mini 2: MSRP $399.99, often $299.99
- eBee VISION: reseller $59,717
- Trinity Pro: Blue Skies Drone Shop $21,260
- Helicopter rates: Healdsburg Tribune (Sonoma Henry 1, ~$4,050/h); LAPD audit ($2,916/h); Thurston County ($1,200/h, projected)
- Civil Air Patrol: 2023 Report to Congress ($170/h blended); CAPR 173-3 supplements (C182 $93/h dry)
- P50B cycle life: NANOGE 2026 abstract (~900 cycles to 80%); maker claim ~1,400
- Mould cost anchor: "So you want to be a composite RC sailplane manufacturer", RC Soaring Digest, 2021
