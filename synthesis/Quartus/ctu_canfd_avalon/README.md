ctu_canfd_avalon_hw.tcl is a script which can be used for creating a component
in Intel Platform Studio. The controller register block is accesible over Avalon bus.

You can use it like this:

 1. Create a release of IP core (scripts/create-release.py)
 2. Copy generated release files here into src/ directory
 3. Now you can optionally copy this directory to where your IP cores for Quartus project are
 4. Open ctu_canfd_avalon_hw.tcl from Intel Platform Studio when creating a new component

So this directory should looks like this:
```
ctu_canfd_avalon
   +-- ctu_canfd_avalon_hw.tcl
   +-- src/
       +-- *.v (release files)
```

**IMPORTANT:** If you are creating your own .tcl or component by hand,
it is critical to set correctly these (besides others) properties of slave
Avalon interface. Compare with the document _CTU CAN FD IP CORE. System Architecture_,
part _2.1 Memory bus_.

```
set_interface_property registers addressUnits SYMBOLS
set_interface_property registers readLatency 1
set_interface_property registers readWaitTime 0
```
