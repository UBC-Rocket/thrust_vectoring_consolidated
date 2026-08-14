# ESC Project Log / Notes

## DATE: August 9, 2026

- Finally understood why 3 phase is more effecient fundamentally, and finally able to feel it make it sense. (Kind of had a idea before but could not feel it)

## DATE: August 10, 2026

- Learnt about stator, rotor, mag fields in a motor, rotation mag field, constant power, math behind V/I/P in a motor, 90 deg between stator and rotor field etc.. (Cleared up some really stupid misunderstandings)
- Cleared up doubts about back emf, how it is used for six step commutation / FOC and RPM, typical back emf plot, zero crossing, relation to flux, how phase currents are used for FOC etc..
- Looked into high level of a typical ESC, design considerations like Dead time, shoot through, thermal, PWM freq considerations, parasitics etc.
- FOC vs Six step trapeziodal, wanted to do FOC cause sounds cooler but it would be stupidly overkill with no benefit so decided on **Six Step Commutation**
- Cleared up some really stupid doubts and mistakes (Took a while to get there)
- Added all the good papers from silicone manufactures on all the topic's above that i used for research to the reference resources folder
- Looked into AM32, compatible hardware options, opensource projects, constraints etc..
- Specced out a FET - **Infineon IAUC120N04S6L005ATMA1**, wowed by this FET, just amazing. Extreamely low RDS(on) - 0.43mR @ VGS >= 10V, $\theta$-ja of just 27, did thermal calc - temp rise of about 18 C only at 40A: **Do not need a heatsink** (Yay)
- Decided on **STM32G070CBT6** MCU, AM32 compatibile, M0+ Arm Cortex, 128Kb flash, 64MHz, LQFP 48 and just 2 Bucks plus good stock in LCSC. Should be good
- Decided on shunt based current monitoring for each phase and for battery
- 