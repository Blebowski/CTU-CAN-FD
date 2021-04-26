# CTU CAN FD

CTU CAN FD is soft IP-Core written in VHDL which supports ISO and NON-ISO versions
of CAN FD protocol.

CTU CAN FD is **NOT a prototype**. RTL is compliant with ISO 1198-1:2015.
It is tested by ISO 16845-1 2016 sequence in regression run **every day**.

RTL is implemented with VHDL-93, synthesizable with most FPGA and ASIC flows.
TB is implemented with VHDL 2008.

## License

CTU CAN FD RTL and TB are published under following license :  
[![License](https://img.shields.io/badge/License--black.svg)]( https://gitlab.fel.cvut.cz/canbus/ctucanfd_ip_core/blob/master/LICENSE)

Commercial usage of RTL and TB requires authors consent.

Linux driver for CTU CAN FD is published under GPLv2 license.

The CAN protocol is developed by Robert Bosch GmbH and protected by patents.
Anybody who wants to implement this IP core on silicon or FPGA for commercial
purposes has to obtain CAN protocol license from Bosch.

## Design

RTL design of CTU CAN FD is independent from vendor specific libraries or macros. It is fully-synchronous design (single clock domain).

FPGA / ASIC target can be selected by top level generic. ASIC target implements clock gating for memories to achieve low dynamic power consumption. Additionally, clock enables are used frequently to allow inferred clock gating on ASIC.

Architecture of CTU CAN FD is described in:  
[![System architecture](https://img.shields.io/badge/System_architecture--blue.svg)]( http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/doc/System_Architecture.pdf)

Functional description of CTU CAN FD is described in:  
[![Datasheet](https://img.shields.io/badge/Datasheet--blue.svg)]( http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/doc/Datasheet.pdf)

## Test-bench

CTU CAN FD has its own VIP (verification IP) with ISO 16845-1 2016 compliance sequence, which is easy to integrate to other
system level test-bench. In addition to ISO 11898-1 compliance, all other features of CTU CAN FD are verified.

There are 3 types of tests available:
- Feature tests (open-source)
- Reference tests (open-source)
- Compliance tests (only binary open-source, source code available upon agreement).

TB has extensive PSL functional coverage, see regression coverage in:
[![Functional coverage](https://img.shields.io/badge/functional%20coverage--orange.svg)](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/regression_results/functional_coverage/functional_coverage_report.html)

GCov is used to collect (rudimentary) code coverage. See regression coverage in:  
[![Code coverage](https://gitlab.fel.cvut.cz/canbus/ctucanfd_ip_core/badges/master/coverage.svg)](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/regression_results/coverage/)

Detailed description of test-bench and CTU CAN FD VIP is in:  
[![Testbench architecture](https://img.shields.io/badge/Testbench--blue.svg)]( http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/doc/Testbench.pdf)

CTU CAN FD is simulated on RTL (no gate level simulations so far).

## How to use it ?

Download delivery package ("public" directory) from: [![Delivery package](https://img.shields.io/badge/Delivery-package--blue.svg)]( https://gitlab.fel.cvut.cz/canbus/ctucanfd_ip_core/-/jobs/artifacts/master/browse?job=pages). Package is created by daily regression on master branch.

Delivery package contains:  
- RTL design
- Tesbench (VIP + Feature and Reference Tests)
- Binary of compliance test library.
- Documentation (Datasheet, System Architecture, Testbench)
- Regression results (with Compliance test results)
- Functional coverage results from regression

RTL CAN be easily integrated into a design with help of [![System architecture](https://img.shields.io/badge/System_architecture--blue.svg)]( http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/doc/System_Architecture.pdf) document.

VIP can be easily integrated into other test-bench with help of [![Testbench architecture](https://img.shields.io/badge/Testbench--blue.svg)]( http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/doc/Testbench.pdf) document. Note that running compliance tests requires access to Compliance test library which is only available with commercial agreement.

## Development tools

To simulate CTU CAN FD, following tools are used:

GHDL, VHDL simulator:  
[GHDL](https://github.com/Blebowski/ghdl).

GTKWave, Waveform viewer:  
[GTKWave](http://gtkwave.sourceforge.net/)

Vunit, VHDL unit test framework, with modifications allowing to start GTKWave interactively when simulation run starts:  
[Vunit](https://github.com/mjerabek/vunit).

Python 3 and following modules: pyvcd attrs jinja2 parsy pyyaml click yattag json2html

There is a docker image which contains all dependencies needed available at:  
[Simulation docker](https://gitlab.com/canfd/server-tools/container_registry).

## Continuous integration

All tests are automated into several regression runs:  
- Fast - No randomization (seed=0), must pass before merge request is merged (basic sanity)
- Nightly - Randomized, runs every night.
- Compliance - Runs ISO 11845-1 2016 compliance test sequence, runs every night

Results of latest test run + logs are available under:  
[![pipeline status](https://gitlab.fel.cvut.cz/canbus/ctucanfd_ip_core/badges/master/pipeline.svg)](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/regression_results/tests_fast.xml)

## Synthesis

CTU CAN FD has been synthesized into Xilinx and Intel FPGAs. BRAMs were inferred for
TX and RX buffers memories. CTU CAN FD reached around 100 MHz of maximal frequency on
Cyclone V/IV and Xilinx Zynq devices.

TODO: Show synthesis benchmark.

## Linux driver

CTU CAN FD has SocketCAN Linux driver which is described in:  
[![Linux driver](https://img.shields.io/badge/Linux_driver--blue.svg)](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/doc/linux_driver/build/ctucanfd-driver.html)

Driver consists from 3 parts:  
- Network
- Platform
- PCI

Driver has been tested on three boards:
- [![PCI board](https://img.shields.io/badge/PCI_board--blue.svg)](https://gitlab.fel.cvut.cz/canbus/pcie-ctucanfd)
- [![Xilinx Zynq board](https://img.shields.io/badge/Zynq_board--blue.svg)](https://gitlab.fel.cvut.cz/canbus/zynq/zynq-can-sja1000-top) (Used for automated tests)
- [![Intel SoC Systems](https://img.shields.io/badge/Intel_SoC--blue.svg)](https://gitlab.fel.cvut.cz/canbus/intel-soc-ctucanfd)

Operation up to 5 Mbits was tested (depending on physical layer transceiver type).

## Linux driver tests

Linux driver was debugged and tested manually against Kvaser devices, CANoe and other CAN FD controllers. Regular communication
and handling of Error states were debugged manually. Automated test for latest IP core version version is run daily at CTU FEE. It pulls latest CTU CAN FD RTL and integrates them into respective Xilinx Zynq FPGA project, compiles latest SocketCAN driver and runs CAN/CAN FD communication (500 Kbit/s Nominal bit rate, 4 Mbit Data bit rate) with randomly generated frames between CTU CAN FDs and FD tolerant SJA1000. Results can be found at:..
[![FPGA Emulator tests](https://img.shields.io/badge/FPGA_Emulator_Tests--cyan.svg)](https://gitlab.fel.cvut.cz/canbus/zynq/zynq-can-sja1000-top/pipelines)

However, there are no written tests for the driver itself (apart from compiling it without error and passing Linux kernels checkpatch which is required for pipeline to pass). In future QEMU + VPCIE + GHDL cosimulation is planned.

## QEMU emulation

The CTU CAN FD IP core functional model is part of QEMU mainline. QEMU CAN documentation [docs/can.txt](https://git.qemu.org/?p=qemu.git;a=blob;f=docs/can.txt) describes CTU CAN FD emulation setup.

