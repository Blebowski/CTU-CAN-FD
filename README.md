# CTU CAN FD IP Core

CAN FD IP Core written in VHDL. Designed to be compliant with ISO1198-1:2015.
Supports ISO and NON-ISO versions of CAN FD protocol.

## License

CTU CAN FD is published under MIT license shown in: [![License](https://img.shields.io/badge/License--black.svg)]( https://gitlab.fel.cvut.cz/canbus/ctucanfd_ip_core/blob/master/LICENSE)

CTU CAN FD linux driver is published under GPLv2 license.

The CAN protocol is developed by Robert Bosch GmbH and protected by patents.
Anybody who wants to implement this IP core on silicon or FPGA for commercial
purposes has to obtain CAN protocol license from Bosch.

## Design

CTU CAN FD RTL is written in VHDL with no vendor specific libraries/components required.
CTU CAN FD is fully synchronous design.

Architecture of CTU CAN FD is described in:
[![System architecture](https://img.shields.io/badge/System_architecture--blue.svg)]( http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/ctu_can_fd_architecture.pdf)

Functional description of CTU CAN FD is in datasheet:
[![Datasheet](https://img.shields.io/badge/Datasheet--blue.svg)]( http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/ProgDokum.pdf)

CTU CAN FD is written with frequent usage of clock enables (FPGA) allowing inferred clock gating.

## Verification

CTU CAN FD verification is done in GHDL + VUnit + GTKWave combination. Verification is done
on RTL (no gate level sims so far...).

Custom GHDL with wave dumping speed-up is used:
[GHDL](https://github.com/Blebowski/ghdl).

Custom Vunit is used which allows starting GTKWave interactively when simulation run starts:
[Vunit](https://github.com/mjerabek/vunit).

There are following types of tests:
    Unit tests - own testbench for each module
    Feature tests - common testbench
    Sanity test - One testbench
    Reference test - One testbench

Test architecture is further described in: TODO.

Each test/testbench has common header which is used to collect what has been verified: TODO.

There is PSL functional coverage and PSL assertions available in: [![functional coverage](https://img.shields.io/badge/functional%20coverage--orange.svg)](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/functional_coverage/functional_coverage_report.html)

For now GCov is used to collect code coverage in GHDL and its results are shown in:
[![coverage report](https://gitlab.fel.cvut.cz/canbus/ctucanfd_ip_core/badges/master/coverage.svg)](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/coverage/)

All tests are automated into two runs:
    - Fast - no randomization (seed=0), must pass before merge request is merged.
    - Nightly - randomized, runs every night.

Results of latest test run + logs are available under:
[![pipeline status](https://gitlab.fel.cvut.cz/canbus/ctucanfd_ip_core/badges/master/pipeline.svg)](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/tests_fast.xml)

At the moment, CTU CAN FD compliance towards ISO1198-1:2015 has not been proven.

## Synthesis

CTU CAN FD has been synthesized into Xilinx and Intel FPGAs. BRAMs were inferred for
TX and RX buffers. CTU CAN FD reached about 100 MHz of maximal frequency. Synthesis
results are shown in: TODO.

## Linux driver

CTU CAN FD has SocketCAN Linux driver which is described in:
[![Linux driver](https://img.shields.io/badge/Linux_driver--blue.svg)](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/driver_doc/ctucanfd-driver.html)

There are three driver parts:
    - network
    - platform
    - PCI

Driver has been tested on two boards:
    - [![PCI board](https://img.shields.io/badge/PCI_board--blue.svg)](https://gitlab.fel.cvut.cz/canbus/pcie-ctu_can_fd)
    - [![Xilinx Zynq board](https://img.shields.io/badge/Zynq_board--blue.svg)](https://gitlab.fel.cvut.cz/canbus/zynq/zynq-can-sja1000-top)

Operation up to 5 Mbits was tested (depending on physical layer transceiver type).

## RTL release package

CTU CAN FD RTL is postprocessed via script which removes PSL assertions and some signals intended
only for verification purposes. RTL release package from latest sources is available in:
TODO

