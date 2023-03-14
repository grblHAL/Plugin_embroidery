## Embroidery plugin

This plugin contains code for streaming embroidery files stored on a SD card.
Currently .pes with embedded .pec format data \(Brother\) and .dst format \(Tajima\) is supported.

__*** Work in progress, not yet available for compilation ***__

The reasons for streaming these formats from SD card instead of first translating to gcode are:

* Translation to gcode is not perfect and it might be hard to get timing perfect.

* The formats are binary and not possible to send to the controller from a "standard" gcode sender.

* More advanced options for controlling the machine is possible.

* ...

In order to support rendering of projects in senders a simple gcode format can be requested from the controller
with the `$F<(filename)` command where `(filename)` is the name of the file to be rendered. The output is plain
gcode terminated with `ok`.  
Note that this output _cannot_ be used to run jobs by streaming them back, `$F(filename)` has to be used!

#### Settings:

TBA

#### Dependencies:

Driver and board with SD card plugin support and one interrupt capable auxillary input.

#### Other options:

grblHAL supports [M66, wait on input](https://linuxcnc.org/docs/2.5/html/gcode/m-code.html#sec:M66-Input-Control),
this may be used for triggering next stitch if gcode translation is to be used.

#### Credits:

---
2023-03-14
