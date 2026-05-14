## Unreleased changes
- CI: Migrated the repository and regression to Github
- Change license owner to Logic Design Services
- Clean `doc` folder, remove legacy unused images
- Make linux driver compile with 6.17 kernel and newer
- *STATUS[RXPE]* with *MODE[RXBAM]=0* now detects parity
  error upon the *RX_DATA* read instead of detecting upon
  moving the RX Buffer read pointer.

## Version 2.7 - 7.1.2026
- RTL: Optimized to remove dead-code and unreachable code
- RTL: Fix frame filters to always pass the Error frame through when IVLD=0.
- RTL: Extend TRV_DELAY to max 256 instead of 128, Extend max SSP position to 512 cycles. Allow up-to 7 bits "in-flight".
- TB: Refactored feature tests -> Massive renaming of feature test functions.
- TB: Split the regression targets into "per-device-config" run -> Small, Medium, Big DUT configs are used.
- TB: Swap to NVC simulator in regression -> Measure code coverage in CI run, reach 100 %
- DOC: Minor fixes based on feedback from community

## Version 2.5.2 - 15.12.2023
- Bug-fix version of 2.5 release
- Fixed incorrect behavior of Bus Monitoring mode with multiple nodes
  on the CAN bus

## Version 2.5.1 - 15.12.2023
- Bug-fix version of 2.5 release
- Fixed reset source of *STATUS[TXPE]* an *STATUS[RXPE]*
- Fixed non-detecting bit error during last bit of CAN CRC
  when the bit error occured after last Secondary sampling point
- Fixed corner-case when bit error occurs during last bit of DLC
  when CTU CAN FD was in Loopback mode, RX buffer would become
  inconsistent

## Version 2.6 - 8.9.2024
- Added Error frame logging to RX Buffer

## Version 2.5 - 15.12.2023
- Added Parity support on RX Buffer and TXT Buffer RAMs
- Added configurable corruption of transmitted frames
- Port to VCS simulator

## Version 2.4 - 28.8.2021
- Refactored testbench to have single top
- Implemented Restricted Operation Mode
- Implemented Time Triggered Transmission mode
- Implemented RX Buffer Automatic mode

## Version 2.3 - 20.2.2021
- Implemented Protocol Exception support (*MODE[PEX]*)
- Implemented Conformance test suite according to ISO1898-1 2015
- Fixed Many bugs towards conformance with ISO1898-1 2015
- Change license to request aproval for using

## Version 2.2 - 5.10.2018
- Remove Tripple Sampling mode
- Added many feature tests
- Many other improvements

## Version 2.2 - 16.10.2019
- Added Linux driver documentation
- Added Linux driver debian build
- Rewrote Protocol control, Bit timing and Bus sampling to ASIC-grade work

## Version 2.1 - 5.10.2018
- Significantly reworked and lowered resource usage on FPGAs


## Version 2.0 - 23.11.2017
- Initial public release under MIT license