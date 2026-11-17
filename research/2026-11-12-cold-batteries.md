# Lodestar in cold weather: battery derating

2026-11-12, US Central. This is a screening estimate; nothing here is measured on the Lodestar pack.

## Answer

Plan winter endurance at about **90% of the warm figure** if the packs are pre-warmed before launch. Use **80%** for a pack left cold in a vehicle before launch. For the compact XPS wing that is **2.8 h or 2.5 h with SiCore**, against 3.07 h warm, and **2.4 h or 2.1 h with P50B**, against 2.67 h. Cold, dense air gives a small amount back. At −10 °C, 2,000 m air is about 4–5% denser than the standard atmosphere, which cuts cruise power by roughly 2%.

## Evidence

- **SiCore (silicon anode).** Amprius states 85% capacity retention at −20 °C versus 25 °C for its cells, without naming a model ([Amprius products](https://amprius.com/products)). An older 3.7 Ah Amprius cell gave about 80% of its energy at −20 °C ([Amprius for NASA, 2023](https://www.nasa.gov/wp-content/uploads/2024/01/03-amprius-technologies-for-nasa-2023stefan.pdf)). Energy is the right measure here because cold also lowers voltage.
- **P50B.** The [Molicel P50B data sheet](https://www.fsaeonline.com/content/Molicel-TDS--INR-21700-P50B-80122.pdf) rates discharge from −40 to 60 °C and plots capacity against temperature. The plotted current isn't printed, and third-party readings of the curve are not reliable enough to quote. I treat P50B like SiCore until a cold discharge test at the Lodestar's rate exists.
- **The Lodestar's discharge is gentle.** A 3 h flight empties the pack at about 0.33C. Cold costs less capacity at low current, because most of the loss is voltage sag across internal resistance. This is why the published low-temperature figures, mostly at 0.2–0.5C, are relevant.

## Why the cells cool down in flight

At 0.33C each cell makes very little heat, of order tens of milliwatts. A whole pack makes about 1 W, which is not enough to keep it warm. The packs sit inside the foam core. A rough estimate takes 10 mm of XPS (conductivity about 0.03 W/m·K) over about 0.2 m² of pack surface and 2.4 kg of cells at about 900 J/kg·K. That gives a cool-down time constant of **roughly an hour**. A pack launched at 25 °C into −10 °C air is near 0 °C after about 1.5 h. It reaches the cold end of the flight, where voltage sag matters most, already cold. Pre-warming therefore helps, but it doesn't remove the loss. That is why the planning figure is 90% and not 100%.

## What would close this

1. Discharge both cell types at 0.33C at 25, 0, −10 and −20 °C to the Lodestar's cutoff voltage, and measure energy (Wh), not capacity (Ah).
2. Fly or chamber-soak an instrumented pack in the wing core to measure its real cool-down time constant.
3. Decide whether a small heater pad (a few watts, about 1% of cruise power) is worth carrying for winter operations. On the estimate above it would hold the pack near 15 °C.
