# Real vehicle definitions

The game's own vehicles (`dynamic/` of the MaSzyna release used for the tests, 2026-10), to test the simulation on the vehicles players drive. Chosen to cover every engine type, brake valve, brake handle, local brake, coupler type, door type and brake delay set the game data uses. `pkp/` is left out of the paths, the other operators keep theirs.

They aren't the game's files: each `.json` here holds what the game's vehicle loader reads of the `.fiz`, with the files it includes, in the form of the tests' own definitions (`vehicle.h`): the values of every section line, and the rows of the lists. `test_fiz.cpp` converts them, using the game's parser for the includes, and checks the loader reads the same of the converted definition as of the game's file (`fiz.h`): the same sections with the same values, the same rows read by the same lists, in the same order. Comments, and lines the loader skips, are left out.

Convert them again, from the game data in `<game>/dynamic`, and add vehicles (paths as in the table, without the extension, separated by commas), with:

    EU07_UNIT_TESTS_FIZ_SOURCE=<game>/dynamic EU07_UNIT_TESTS_FIZ_ADD=sm42_v1/6d1 eu07_unit_tests -tc="fiz: conversion*"

Every `.json` here is a vehicle picked up by `test_fiz.cpp`: record the parameters of an added one with `EU07_UNIT_TESTS_UPDATE_GOLDEN=1`.

| Path | Vehicle | Engine | Brake valve | Brake handle | Local brake | Coupler |
|---|---|---|---|---|---|---|
| `11xa_v2/93a.fiz` | 11XA | - | ESt4 | - | ManualBrake | Screw |
| `14xa_v1/173a.fiz` | 14XA | - | ESt4HBG300-s216 | - | ManualBrake | Screw |
| `168a_v1/168a.fiz` | 168A | - | ESt4HBG300 | - | ManualBrake | Screw |
| `21xm_v1/218ma_a_kd.fiz` | 218M | DieselEngine | EStED | MHZ_K5P | PneumaticBrake | Screw |
| `21xm_v1/214ma.fiz` | 214M | DieselEngine | EStED | MHZ_K5P | PneumaticBrake | Screw |
| `3wl_v2/3wl.fiz` | 3WL | - | ESt3 | - | ManualBrake | Screw |
| `430w_v2/430w.fiz` | 430W | - | ESt3AL2HBG300 | - | ManualBrake | Screw |
| `4e_v1/4e.fiz` | 4E | ElectricSeriesMotor | LSt | FV4a | PneumaticBrake | Screw |
| `bdhpumn_v3/bdhpumn.fiz` | BDHPUMN | - | KE | - | ManualBrake | Screw |
| `bhp_v1/bhp-b.fiz` | BHP | - | ESt3AL2 | - | ManualBrake | Articulated |
| `br285_v1/br285.fiz` | BR285 | DieselElectric | KE | MHZ_6P | PneumaticBrake | Screw |
| `cd/810_v2/810.fiz` | 810 | DieselEngine | ESt3_PZZ | FV4a | PneumaticBrake | Screw |
| `cd/754_v1/754.fiz` | 754 | DumbDE | LSt | FV4a | PneumaticBrake | Screw |
| `dr2_v1/dr2.fiz` | DR2 | WheelsDriven | - | - | ManualBrake | None |
| `e186_v2/eu47.fiz` | E186 | ElectricInductionMotor | EStED | MHZ_6P | PneumaticBrake | Screw |
| `e6act_v2/e6acta.fiz` | E6ACT | ElectricInductionMotor | EStED | MHZ_K5P | PneumaticBrake | Screw |
| `ed72_v1/5bs-rb.fiz` | ED72 | - | EP2 | FVel6 | ManualBrake | Automatic |
| `elf_v1/22wec-a.fiz` | ELF 22WEc | ElectricInductionMotor | EStED | MHZ_K5P | PneumaticBrake | Automatic |
| `elf_v1/21we-a.fiz` | ELF | ElectricInductionMotor | EStED | MHZ_K5P | PneumaticBrake | Automatic |
| `en57_v1/5bgb.fiz` | EN57 | - | EP2 | FVel6 | ManualBrake | Automatic |
| `en57_v1/6ba_840.fiz` | EN57 | - | EP2 | FVel6 | ManualBrake | Automatic |
| `en57_v1/6bai.fiz` | EN57 | - | K | St113 | ManualBrake | Automatic |
| `en57akl_v1/akl_ra.fiz` | EN57AKL | ElectricInductionMotor | EStED | MHZ_EN57 | PneumaticBrake | Automatic |
| `ep09_v2/104e.fiz` | EP09 | ElectricSeriesMotor | EStED | FV4a | PneumaticBrake | Screw |
| `et22_v3/201e-rn.fiz` | ET22 | ElectricSeriesMotor | LSt | FV4a | PneumaticBrake | Screw |
| `et22_v3/201e-w.fiz` | ET22 | ElectricSeriesMotor | LSt | FV4a | PneumaticBrake | Screw |
| `et42_v2/112e-a.fiz` | ET42 | ElectricSeriesMotor | LSt | FV4a | PneumaticBrake | Screw |
| `et42_v2/112e-b.fiz` | ET42 | ElectricSeriesMotor | LSt | FV4a | PneumaticBrake | Screw |
| `eu04_v1/eu04-01.fiz` | EU04 | ElectricSeriesMotor | K | Knorr | PneumaticBrake | Screw |
| `ew58_v1/ew58-d.fiz` | EW58 | - | EP1 | - | ManualBrake | Automatic |
| `ew58_v1/ew58-sa.fiz` | EW58 | ElectricSeriesMotor | EP1 | FVE408 | ManualBrake | Automatic |
| `impuls_v1/31we-a_pr.fiz` | IMPULS | ElectricInductionMotor | EStED | MHZ_6P | PneumaticBrake | Automatic |
| `road/man/nl293.fiz` | MAN (road) | DieselEngine | - | - | HydraulicBrake | Bare |
| `sa134_v1/218mc_a.fiz` | SA134 | DieselEngine | EStED | MHZ_K5P | PneumaticBrake | Screw |
| `sieciowiec_v1/a.fiz` | SIECIOWIEC | - | - | - | NoBrake | Screw |
| `sm03_v1/sm03.fiz` | SM03 | DieselEngine | K | Knorr | PneumaticBrake | Screw |
| `sm31_v1/411d.fiz` | SM31 | DumbDE | LSt | FV4a | PneumaticBrake | Screw |
| `sm40_v1/sm40.fiz` | SM40 | DieselElectric | K | Knorr | PneumaticBrake | Screw |
| `sm42_v1/6d1.fiz` | SM42 | DumbDE | LSt | FV4a | PneumaticBrake | Screw |
| `sn61_v2/sn61.fiz` | SN61 | DieselEngine | ESt3d_PZZ | FV4a | PneumaticBrake | Screw |
| `st44_v2/m62-1151.fiz` | ST44 | DieselElectric | LSt | FV4a | PneumaticBrake | Screw |
| `st45_v2/301dd.fiz` | ST45 | DumbDE | SW | MHZ_K8P | PneumaticBrake | Screw |
| `static/sug/sug_kielce.fiz` | SUG | ElectricSeriesMotor | W | Knorr | PneumaticBrake | Screw |
| `su45_v2/301d.fiz` | SU45 | DumbDE | LSt | FV4a | PneumaticBrake | Screw |
| `su46_v2/303d.fiz` | SU46 | DumbDE | LSt | FV4a | PneumaticBrake | Screw |
| `tem2_v2/tem2a.fiz` | TEM2 | DieselElectric | LSt | M394 | PneumaticBrake | Screw |
| `wm15_v1/wm15.fiz` | WM15 | Dumb | LSt | FV4a | PneumaticBrake | Screw |
| `wmb10_v1/wmb10.fiz` | WMB10 | DieselEngine | LSt | FV4a | PneumaticBrake | Screw |
| `z1_v2/152a.fiz` | Z1 | - | KE | - | ManualBrake | Screw |

The paths are those of the game's `.fiz` files. `elf_v1/inverter.fiz` is a part the Elf units include, not a vehicle. The 218M (V2NList), the SA134 (HTCList) and the 22WEc (ffBrakeList) are there for lists the others lack. The 214MA stands for the 214M railbus, whose own file repeats a section (TurboPos and MotorParamTable twice), which the definitions here can't hold.
