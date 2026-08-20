# ESC Project Log / Notes

## DATE: August 9, 2026

- Finally understood why 3 phase is more effecient fundamentally, and finally able to feel it make it sense. (Kind of had a idea before but could not feel it)

## DATE: August 10, 2026

- Learnt about stator, rotor, mag fields in a motor, rotation mag field, constant power, math behind V/I/P in a motor, 90 deg between stator and rotor field etc.. (Cleared up some really stupid misunderstandings)

- Cleared up doubts about back emf, how it is used for six step commutation / FOC and RPM, typical back emf plot, zero crossing, relation to flux, how phase currents are used for FOC etc..

- Looked into high level of a typical ESC, design considerations like Dead time, shoot through, thermal, PWM freq considerations, parasitics etc.

- FOC vs Six step trapeziodal, wanted to do FOC cause sounds cooler but it would be stupidly overkill with no benefit so decided on **Six Step Commutation**

- Cleared up some really stupid doubts and mistakes (Took a while to get there)

- Added all the good papers from silicon manufactures on all the topic's above that i used for research to the reference resources folder

- Looked into AM32, compatible hardware options, opensource projects, constraints etc..

- Specced out a FET - **Infineon IAUC120N04S6L005ATMA1**, wowed by this FET, just amazing. Extreamely low RDS(on) - 0.43mR @ VGS >= 10V, $\theta$-ja of just 27, did thermal calc - temp rise of about 18 C only at 40A: **Do not need a heatsink** (Yay). Some stock in LCSC and little expensive but worth it

- Decided on **STM32G070CBT6** MCU, AM32 compatibile, M0+ Arm Cortex, 128Kb flash, 32Kb RAM, 64MHz, LQFP 48 and just 2 Bucks plus good stock in LCSC. Should be good

- Decided on shunt based current monitoring for each phase and for battery

- Looked into TVS Diodes and Decoupling caps at Vbat and phase out to handle proper decoupling, dv/dt spikes, inductive kickback (L*di/dt), back-emf and phase voltage adding/subracting, speccing for worst case, FET VDSmax global limit, regenerative braking - charging the lipo with it, braking circuit, braking strength, back emf adding in braking phase and subracting in drive phases

- Decided on low ESR/ESL decoupling cap network (probably low ESR polymer tantalum - KEMET) at Vbat phase inputs for local reservoir, smooth out the ripple / spikes and enable fast switching, TVS at Vbat phase input for ESD to protect lipo/FET VDSmax from voltage spikes and TVS at phase output for ESD and extra clamping

- Combining the above with low braking strength(Reduce Duty cycle slowly) - set in AM32 firmware to reduce parasitics and keep the regen. current low and **Important to spec TVS and Caps to Vphase + Back-emf + overshoot (worst case)**

- Clarified Dead time control vs Shoot through protection vs interlock, aiming for Dynamic dead-time control plus shoot through protection (Miller clamp or Strong pull down) to handle dv/dt FET turning on or interlock feature
 
**Summary** :

- Deepdive on ESC fundamentals and developed decent understanding of it

- Finalized high level architechture such as Six step vs FOC etc..

- Specced out the main components

- Deepdive into portection and parasitcs handling circuits

## DATE: August 11, 2026

- Revision of everything learnt yesterday especially question i had, mistakes i made, misunderstandings etc..

- Looked into freewheeling and using FET's body diode to freewheel Inductive kick back (very intresting) how opposite FET body diode forward biases, how it happens during dead time, timeline of events etc...

## DATE: August 12, 2026

- Cleared up little cofusion about bootstrap cap, how it sits between BST and SW node - charged to 12V/Driver Voltage when low and VCC + 12V when SW node is high and that BST is connected to high side fet gate to keep it on

- Found some repos on open source ESC designs and just messing around for reference

- Started search for gate driver IC, looked through Infineon, TI, Analog Devices and some other company products and factoring in LCSC price and stock, finalized either a DRV series IC (TI) or two infineon IC options. 

## DATE: August 13, 2026

- Decided on **DRV8353SRTAR** Gate Driver cause well know chip series, Has all the specs needed, cheap and good stock in LCSC and TI's smart gate architechture makes design easier. 

- Added Footprints for FET and MCU in our altium library

## Date: August 15, 2026

- Added Footprint for Gate Driver in our altium library

## Date August 16, 2026

- Looked into how to spec switching frequency for PWM and target rise and fall time, learnt that higher freq. / lower rise time means smoother drive but more switching losses and vice-versa. General ESC PWM freqency is between 25-96 KHz, with 48 KHz being generally used value. Thinking of just using 48 KHz, and **looking into rise and fall time calculation, need to learn a few things first.**

- 


