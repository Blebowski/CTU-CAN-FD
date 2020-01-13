# CTU CAN FD IP Core

CAN FD IP Core written in VHDL. Designed to be compliant with ISO1198-1:2015.
Supports ISO and NON-ISO versions of CAN FD protocol.

## License

CTU CAN FD is published under MIT license shown in:  
[![License](https://img.shields.io/badge/License--black.svg)]( https://gitlab.fel.cvut.cz/canbus/ctucanfd_ip_core/blob/master/LICENSE)

CTU CAN FD linux driver is published under GPLv2 license.

The CAN protocol is developed by Robert Bosch GmbH and protected by patents.
Anybody who wants to implement this IP core on silicon or FPGA for commercial
purposes has to obtain CAN protocol license from Bosch.

## Design

CTU CAN FD RTL is written in VHDL with no vendor specific libraries/components required.
CTU CAN FD is fully synchronous design. CTU CAN FD is written with frequent usage of clock enables to allow inferred clock gating on ASIC.

Architecture of CTU CAN FD is described in:  
[![System architecture](https://img.shields.io/badge/System_architecture--blue.svg)]( http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/ctu_can_fd_architecture.pdf)

Functional description of CTU CAN FD is in datasheet:  
[![Datasheet](https://img.shields.io/badge/Datasheet--blue.svg)]( http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/Progdokum.pdf)

## Dependencies

To simulate CTU CAN FD following dependencies are needed:

GHDL, VHDL simulator with custom changes:  
[GHDL](https://github.com/Blebowski/ghdl).

GTKWave, Waveform viewer:  
[GTKWave](http://gtkwave.sourceforge.net/)

Vunit, VHDL unit test framework, with modifications allowing to start GTKWave interactively when simulation run starts:  
[Vunit](https://github.com/mjerabek/vunit).

Python 3 and following modules: pyvcd attrs jinja2 parsy pyyaml click yattag json2html

For simulation environment, there is a docker image available at:  
[Simulation docker](https://gitlab.com/canfd/server-tools/container_registry).


## Verification

CTU CAN FD verification is done in GHDL + VUnit + GTKWave combination. Verification is done
on RTL (no gate level sims so far...).

There are following types of tests:  
- Unit tests - Verify functionality of some modules stand-alone. Testbench for each module.
- Feature tests - Verify features of CTU CAN FD. Common testbench for many test-cases
- Sanity test - Real bus simulation (with delays, transceiver, noise). One testbench, different configurations.
- Reference test - Verify reception from reference CAN controller. One testbench, different file sets.

Test architecture is further described in:  
TODO.

Each test/testbench has common header which is used to collect what has been verified (call it VRM if you want...):  
[![Verification items](https://img.shields.io/badge/Verification_Items--yellow.svg)(http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/VRM.html)

There is PSL functional coverage and PSL assertions available in:  
[![functional coverage](https://img.shields.io/badge/functional%20coverage--orange.svg)](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/functional_coverage/functional_coverage_report.html)

For now GCov is used to collect code coverage in GHDL and its results are shown in:  
[![coverage report](https://gitlab.fel.cvut.cz/canbus/ctucanfd_ip_core/badges/master/coverage.svg)](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/coverage/)

All tests are automated into two runs:  
- Fast - no randomization (seed=0), must pass before merge request is merged.
- Nightly - randomized, runs every night. Should be debugged before release.

Results of latest test run + logs are available under:  
[![pipeline status](https://gitlab.fel.cvut.cz/canbus/ctucanfd_ip_core/badges/master/pipeline.svg)](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/tests_fast.xml)

At the moment, CTU CAN FD compliance towards ISO1198-1:2015 has not been proven.
It is intended for future to build testbench with test sequences described in ISO16845-1:2016.

## Synthesis

CTU CAN FD has been synthesized into Xilinx and Intel FPGAs. BRAMs were inferred for
TX and RX buffers. CTU CAN FD reached about 100 MHz of maximal frequency. Synthesis
results are shown in:  
TODO.

## Linux driver

CTU CAN FD has SocketCAN Linux driver which is described in:  
[![Linux driver](https://img.shields.io/badge/Linux_driver--blue.svg)](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/driver_doc/ctucanfd-driver.html)

There are three driver parts:  
- Network
- Platform
- PCI

Driver has been tested on two boards:
- [![PCI board](https://img.shields.io/badge/PCI_board--blue.svg)](https://gitlab.fel.cvut.cz/canbus/pcie-ctu_can_fd)
- [![Xilinx Zynq board](https://img.shields.io/badge/Zynq_board--blue.svg)](https://gitlab.fel.cvut.cz/canbus/zynq/zynq-can-sja1000-top) (Used for automated tests)

Operation up to 5 Mbits was tested (depending on physical layer transceiver type).

## Linux driver tests

Linux driver was debugged and tested manually against Kvaser devices, CANoe and other CAN FD controllers. Regular communication
and handling of Error states were debugged manually. There is an automated emulator at CTU FEE pulling latest CTU CAN FD
RTL, synthesizing into Xilinx Zynq FPGA, running PetaLinux, loading SocketCAN driver and running CAN/CAN FD communication (500 Kbit/s Nominal bit rate, 4 Mbit Data bit rate) with randomly generated frames between CTU CAN FDs and FD tolerant SJA1000. Results can be found at:  
[![FPGA Emulator tests](https://img.shields.io/badge/FPGA_Emulator_Tests--cyan.svg)](https://gitlab.fel.cvut.cz/canbus/zynq/zynq-can-sja1000-top/pipelines)

However, there are no written tests for the driver itself (apart from compiling it without error and passing Linux kernels checkpatch which is required for pipeline to pass). In future QEMU + VPCIE + GHDL cosimulation is planned.

## RTL release package

CTU CAN FD RTL is postprocessed via script which removes PSL assertions and some signals intended only for verification purposes. This is done to save simulation time in systems where CTU CAN FD is integrated (there are hundreds of PSL cover points and PSL assertions). RTL release package from latest sources is available in:  
TODO

