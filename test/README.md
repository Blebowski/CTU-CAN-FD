# Testing Framework

Features:
 * based on VUnit
 * supports Modelsim and GHDL+gtkwave
 * test suite configuration in YAML files
 * assigning TCL and/or GHW waveform layout files to testcases
 * and probably more

## Configuration options

 * `wave`: a path to a TCL file with waveform layout definition, used for
           Modelsim and gtkwave (if not overriden by `gtkw`). For gtkwave, it
           is internally converted to a gtkw file on each run (so modify the
           TCL file, not the generated gtkw).
 * `gtkw`: a path to GTKW file with waveform layout definition for gtkwave; if
           set together with `wave`, this takes precedence. The specified gtkw
           file is not modified.
 * `dump_all_signals`: If true, dump all signals in GUI mode, not only these
                       included in the layout file. May be overriden by
                       `--dumpall` commandline option. By default, it is set to
                       true, but for long-lasting tests with lots of signals it
                       may be necessary to set it to false to prevent `gtkwave`
                       to run out of memory while loading waveforms.
 * many more

## Using waveform layout files

* Specify the file in YML config, either as `gtkw` or `wave` (tcl). Later, this
  might be extended to native gtkw-generating python files.
* Run the tests with `--create-ghws`. This generates signal and type hierarchy.
  You should run this each time you modify a signal in the layout (or add a
  signal both to code and to layout).
* Run in gui mode, using the VUnit `-g` flag.
* If a layout file is specified and `dump_all_signals` is false (and
  `--dumpall` is not used), only the signals specified in the layout file are
  dumped.

# How it works

## Converting Modelsim TCL layout files to GTKW

In addition to setting GHW file directly, it is possible to specify a Modelsim
TCL layout file (with `add wave` commands), which is automatically converted to
GTKW. This has several layers, as described below.

### Interpreting the TCL file

Python comes with TCL interpreter in the standard Tkinter module. The layout
files are thus processed by full-fledged TCL interpreter. The `add wave`
function is then implemented in Python.

This is implemented in `gtkwave.py`.

### Implementing `add wave` function

This function parses the arguments to `add wave` and creates a GTKW file using
`gtkw.GTKWSave` class from the package `pyvcd`. Among its features are:
  * setting display format (dec, hex, bin, signed, ...)
  * setting color (only a limited palette of ~8 colors is supported by gtkwave.
    See `gtkwave.TclFuncs.conv_color` for details).
  * Grouping
  * Delimiters
  * Include all items of a record type in a group (optionally expanded by
    default).

As gtkwave supports displaying just a subset of vector's bits, it requires the
(vector) signal to specify the full bit range, which is not included in the
Modelsim TCL files. This information has to be obtained automatically.

### Getting signal hierarchy and type hierarchy

To detect the bitwidth of a vector, we need to be able to find the signal by its
fully qualified name (FQN) in the design. Furthermore, as there may be a vector
inside of a record type, even type hierarchy must be known.

This information is available in GHDL's GHW data dumps (analogue of VCD). We are
now faced with two problems: how to generate the file, and how to parse it.

#### Generating GHW files

The GHW file is generated when GHDL runs the test case. However, we need it
*before* we run the testcase, to be able to produce the GTKW file for gtkwave
(in `test_common.TestsBase.add_modelsim_gui_file`).

The GHW files are thus generated manually by running the test framework with
`--create-ghws` argument. It could be automated, but the reasoning is that the
relevant signals do not change very often, and the generaing takes non-trivial
amount of time (~2 sec *per testcase*).

Now what does `--create-ghws` do:
1. Append `--elaborate` to VUnit arguments. This causes the test cases to be
   elaborated (GHDL runs and generates the GHW file), but not executed.
2. For each testbench, add GHDL simulation option `--wave=xxx.ghw`, which tells
   GHDL to output the GHW file.

#### Parsing GHW files

GHW files are binary. Non-standard. Not documented. With non-stable format. Only
2 known implementations exist -- one in GHDL (write), one in gtkwave (read).
Fortunately, gtkwave comes with a useful program `ghwdump`, which can output
both signal hierarchy (with `-h` flag) and types (`-t` flag). The output of
`ghwdump` is then parsed in Python (package `testfw.ghw_parse`).

##### Signal hierarchy

Parsing signal hierarchy is very simple, because it is structured as a tree,
with levels differentiated by indentation.
```
design
 instance tb_sanity:
  instance t_sanity:
   port-out errors: natural: #18
   signal error_ctr: natural: #21
```

#### Type hierarchy

Types are printed as VHDL code. The parsing is thus a bit harder and uses a
real parser. The `parsy` Python module, modelled after Haskell's `parsec`, is
used. A little preprocessing is needed to fix some bohus `ghwdump` output. The
implementation probably does not understand all valid VHDL type definitions, and
probably accepts some non-valid ones. It is, however, not needed to implement a
conformant VHDL parser, but to Just Make It Work In Our Case™.
