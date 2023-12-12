# CTU CAN FD

CTU CAN FD is soft IP-Core written in VHDL which supports ISO and NON-ISO versions
of CAN FD protocol.

CTU CAN FD is not a prototype. RTL is compliant with ISO 1198-1:2015.
CTU CAN FD is tested by ISO 16845-1 2016 sequence in regression run on
every merged request.

RTL is implemented in VHDL-93, synthesizable with most FPGA and ASIC flows.
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

RTL design of CTU CAN FD is independent from vendor specific libraries or macros.
It is fully-synchronous design (single clock domain).

FPGA / ASIC target can be selected by top level generic. ASIC target implements
clock gating for memories to achieve low dynamic power consumption. Additionally,
clock enables are used frequently to allow inferred clock gating on ASIC.
Last but not least, RTL is DFT insertion ready and contains additional support
for manufacturing testability.

Architecture of CTU CAN FD is described in:
[![System architecture](https://img.shields.io/badge/System_architecture--blue.svg)]( http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/doc/System_Architecture.pdf)

Functional description of CTU CAN FD is described in:
[![Datasheet](https://img.shields.io/badge/Datasheet--blue.svg)]( http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/doc/Datasheet.pdf)


## Test-bench

CTU CAN FD has its own test-bench and VIP (verification IP) with
ISO 16845-1 2016 compliance sequence. In addition to ISO 11898-1
compliance, all other features of CTU CAN FD are verified.

There are 3 types of tests available in CTU CAN FD test-bench:
- Feature tests (open-source, VHDL)
- Reference tests (open-source, VHDL)
- Compliance tests (open-source, C++ library linked to simulation, [ISO16845 Compliance library](https://gitlab.com/Ondrej_Ille/iso-16845-compliance-tests)).

TB has extensive PSL functional coverage, see regression coverage in:
[![Functional coverage](https://img.shields.io/badge/functional%20coverage--orange.svg)](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/regression_results/functional_coverage/functional_coverage_report.html)

Description of test-bench and CTU CAN FD VIP is in:
[![Testbench architecture](https://img.shields.io/badge/Testbench--blue.svg)]( http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/doc/Testbench.pdf)

CTU CAN FD is simulated as RTL and as post-synthesis gate-level
netlist with Xilinx UNISIM library (unit delay simulation).

See the instructions below in "How to run CTU CAN FD testbench" subsection.


## Regressions (Continuous integration)

All tests are automated into several regression runs:
- Fast ASIC - DUT configuration for ASIC, short run
- Fast FPGA - DUT configuration for FPGA, short run
- Compliance short - Runs majority of ISO 11845-1 compliance test sequence

- Nightly - DUT configuration for ASIC with different DUT parametrizations
- Compliance typ - Runs all ISO 11845-1 tests with "typical" bit-rate on CAN bus.
- Compliance max - Runs all ISO 11845-1 tests with "maximal" bit-rate on CAN bus.
- Compliance min - Runs all ISO 11845-1 tests with "minimal" bit-rate on CAN bus.

- Gate level simple - Runs most of feature tests on gate level netlist as DUT
- Gate level compliance - Runs most of ISO 11845-1 tests on gate level netlist as DUT.

Short summary of results from last regression (including all test types) can be downloaded from:
[Regression summary](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/regression_results/regression_summary.gz)


## Synthesis

CTU CAN FD has been synthesized into Xilinx and Intel FPGAs on several FPGA families
(Zynq, Cyclone IV/V, Spartan). BRAMs are inferred for TX and RX buffers memories.
Maximal reachable frequency is around 100 MHz on Cyclone V/IV and Xilinx Zynq devices.

Daily CI pipeline executes example synthesis on Xilinx Zynq device with 3 different
device configurations:

#### Minimal:

2 TXT Buffers, 32 word RX Buffer, no frame filters
Results: [Area](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/synthesis/minimal_design_config/utilization.rpt)
[Timing](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/synthesis/minimal_design_config/timing_summary.rpt)

#### Typical:

4 TXT Buffers, 128 word RX Buffer, only one bit filter
Results: [Area](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/synthesis/typical_design_config/utilization.rpt)
[Timing](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/synthesis/typical_design_config/timing_summary.rpt)

#### Maximal:

8 TXT Buffers, 1024 word RX Buffer, all frame filters
Results: [Area](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/synthesis/maximal_design_config/utilization.rpt)
[Timing](http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/synthesis/maximal_design_config/timing_summary.rpt)

Note that design is constrained to 100 MHz with no timing violations, combinatorial loops
or latches inferred (FPGA config without clock gate is used). These results together with
gate level simulations (see "Test-bench" above), provide good indicator of high-quality RTL design.


## How to integrate CTU CAN FD RTL ?

1. Compile files from `src/slf_rtl.yml` YAML file into `ctu_can_fd_rtl` VHDL library
in the order files are present in `src/slf_rtl.yml`.
2. Integrate `can_top_level` entity in your design. See [![System architecture](https://img.shields.io/badge/System_architecture--blue.svg)]( http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/doc/System_Architecture.pdf) for details of CTU CAN FD interface.


## How to run CTU CAN FD testbench ?

There are two options to run CTU CAN FD regression:
1. With VUnit + GHDL
2. With Synopsys VCS

Each regression run corresponds to a target from `sim/ts_sim_config.yml` file. There are
following targets available:
   - tb_rtl_test_fast_asic
   - tb_rtl_test_fast_fpga
   - tb_rtl_test_nightly
   - tb_rtl_test_compliance_short
   - tb_rtl_test_compliance_full_typ
   - tb_rtl_test_compliance_full_min
   - tb_rtl_test_compliance_full_max

### Running with GHDL

In CTU CAN FD repository root:

1. `./run-docker-test` - This will pull and launch docker image with GHDL, Vunit, CMake and C Compiler.
2. `cd main_tb/iso-16845-compliance-tests`
3. `./build.sh` - This builds compliance tests library
4. `cd ../..`
5. `./run.py <TARGET_NAME>` To run all tests from `<TARGET_NAME>` target.

If you run `./run.py <TARGET_NAME> --list` you will get list of all available tests
for given target.

## Running with VCS

To run with VCS, you will need Tropic Square HW simulation flow, to get it do following:

1. ``git clone https://github.com/tropicsquare/ts-hw-scripts``
2. ``export PATH=`pwd`/ts-hw-scripts/scripts:$PATH``
3. Make sure you have all Python dependencies required to run `ts-hw-scripts`. See README of `ts-hw-scripts`.

Then, you need `cmake` (version 3.5 or higher) and a C compiler with C++17 support
(e.g. GCC 7.2.0 or higher).

Then in CTU CAN FD repository build compliance tests:
1. ``export TS_REPO_ROOT=`pwd` `` - Sets an important environment variable for simulation flow.
2. ``cd test/main_tb/iso-16845-compliance-tests``
3. ``./build.sh`` - This builds compliance tests library
4. ``export LD_LIBRARY_PATH=`pwd`/build/Debug/src/cosimulation`` - Makes compliance library visible for VCS
5. ``cd $TS_REPO_ROOT``
6. ``ts_sim_run.py --recompile --clear <TARGET_NAME> /*``

If you run ``ts_sim_run.py --recompile --clear <TARGET_NAME> --list-tests``` you will get list
of available tests for given target.


## How to integrate CTU CAN FD VIP (experimental) ?

CTU CAN FD testbench is a stand-alone testbench with `ctu_can_fd_tb_top` as top level
simulated entity. All testing/verification functionality is however implemented in
CTU CAN FD VIP (`ctu_can_fd_vip` entity). It is possible to integrate CTU CAN FD
VIP also to your custom testbench if you connect the VIP to DUT via VHDL hierarchical
references (this is similar to binding System Verilog interfaces in UVM TB).

VIP can be integrated into other test-bench with help of [![Testbench architecture](https://img.shields.io/badge/Testbench--blue.svg)]( http://canbus.pages.fel.cvut.cz/ctucanfd_ip_core/doc/Testbench.pdf) document.

To compile the VIP / TB in such scenario, you first need to compile RTL
(testbench depends on RTL). Then:

1. Compile files from `test/slf_tb_dependencies_simple.yml`
2. Compile files from `test/slf_tb_common.yml`
3. Integrate `ctu_can_fd_vip` into your TB, and connect its interface to CTU CAN FD in design.


## License disclaimer

If you like the design, and would like to use-it, let us know. We are looking for co-operation,
especially on ISO certification which is a big step that CTU CAN FD would like to take. Please,
respect the license, and dont use the RTL and TB in your commercial device without a license contract.


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

The CTU CAN FD IP core functional model is part of QEMU mainline.
QEMU CAN documentation [docs/system/devices/can.rst](https://www.qemu.org/docs/master/system/devices/can.html)
includes section about CTU CAN FD emulation setup.


## Roadmap

There are several options for further development:
- Linux driver testing (QEMU + VPCIE)
- Splitting design into two clock domains
- Support of TTCAN protocol
- Support of CAN XL protocol
- Other features

