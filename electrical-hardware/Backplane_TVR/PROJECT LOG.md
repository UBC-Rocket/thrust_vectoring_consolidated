# Backplane TVR Project Log

## Date: 09/08/2026

- Learnt about USB PD protocol - Sink/Source/DRP negotiations, detailed process from the moment the cable is attached step by step how who is source/sink is decided, trySRC and trySNK, UFP and DFP, how each device records its own CC1/2 states and if it detects that CC1/2 state is different than recorded, then assumes a device in the other end and configures itself as opposite role

- Also about DRP toggling behaviour, how DRP to DRP resolves into role by randomly assigning a role due to toggling and then negotiations begin for proper role where things like trySRC etc.. factor in, how asymetric duty cycle is used in toggling to help resolve and worst case of perfect synchronization when cable is plugged in will still resolve due to clock drift etc.. Beautiful Protocol

- Looked through TI's PD chips (They are known to have good series), deepdive on a few datasheets and stock in LCSC etc..

- Thinking about the PD to Charger architecture

## Date: 10/08/2026

- Decided on TPS25751 PD chip from TI, USB 3.2, DRP compatable, 20v 5A PD/PPS source and sink, Interfaces with some TI battery charger IC's natively and controls it autonomously (Easy integration), integrated power path for PPS (20v 5A) - no external FET's needed, decent stock in LCSC and only 2.5 CAD

- Added PD chip footprint to altium library

## Date: 15/08/2026

- Learnt about official USB PD 3.2 specifications from USB-IF website, parameters like max snkbulkcap before and after PD contract, max D+/- cap, CC1/2 cap limit, some timing requirements etc..

- Worked on PD IC schematics, added all the required footprints in altium library.

- Looked into the TI battery charger IC's that natively interface with PD IC, decided on couple of options that could work

- Decided that MCU will interface with PD IC for commands, config and some telemetry and PD IC will interface with Charger IC and control it natively

- Finalized Battery Charge IC: BQ25756, cause it supports upto 14s Li-ion, 4 FET buck-boost synchronus Bi-Directional, external FET so supports upto 20A, natively interfaces with PD chip and USB PD, input and output current sensing, good range of protection and monitoring features, JEITA compliant temp profile, good stock on LCSC and decent price of 5.3 CAD.

- Added Battery Charger IC to altium library

## Date: 16/08/2026

- Clarified some doubts about TVS, how it operates in negative voltage, UNI sv BI - UNI acts like a regular diode under 0V and forward bias after 0.7V so better for common mode signals and Diff pairs need BI TVS to allow negative voltage.

- Finalized that VBUS needs a USB rated UNI TVS cause no negative voltage so UNI better, CC1/2 needs a 5V signal rated UNI TVS cause common mode signal and D+/- needs a 3.3V rated BI-TVS cause diff pair 

- **Important:** Remember that VBUS/PPHV bus capcitance is VBUS Cap + PPHV cap + Charger IC Vin Cap and that it needs to be less than 100uF after PD contract if no inrush limiting

- Going to use Safe Mode for PD IC PPHV states, PPHV sink path and USB PD disabled by default until the config file (Patch Bundle) is loaded (By FC), PD IC controls the battery charger IC autonomously and recommended mode by TI when External MCU/EEPROM loads the config file and PC IC controls battery charger IC.

- Finished PD IC schematic, add all calc. and design decision explanation in sheets and made it look as pretty as i can

## Date: 



