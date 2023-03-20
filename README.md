## Embroidery plugin

This plugin contains code for streaming embroidery files stored on a SD card.
Currently .pes with embedded .pec format data \(Brother\) and .dst format \(Tajima\) is supported.

__*** Work in progress, experimental ***__

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

These are the initial settings, some are experimental and _all_ setting numbers will change in a final version: 

`$450` - stitch feedrate in mm/min.

`$451` - Z travel. For those who have a stepper motor controlling the needle. _Not yet implemented - feedback required!_

`$452` - trigger aux in port. Port number to use for the needle position sensor. Use `$pins` to check which ports are available.

`$453` - sync mode. If set to `1` the trigger port signal is used to trigger stitch xy-motion. If set to `0` needle control is by stepper motor, see above.

`$454` - stop delay. May be used to set a delay in ms from the second last stitch until the needle motor is switced off.

`$455` - trigger edge. Edge of trigger signal used to start the next stitch move. `0` - falling edge, `1` - rising edge.

`$456` - if set to `1` output a logical one on aux port 0 when the controller is in cycle mode \(xy moving\). This setting is disabled if no aux port is available.  
Can be useful for checking timing of movements vs. the trigger signal with an oscilloscope or a logic analyzer.

#### Dependencies:

Driver and board with SD card plugin support and one interrupt capable auxillary input.

#### Extensions:

Plugin code can register handlers for thread trim and thread \(color\) changes. These can then be used for implementing automatic operation. 

#### Other options:

grblHAL supports [M66, wait on input](https://linuxcnc.org/docs/2.5/html/gcode/m-code.html#sec:M66-Input-Control),
this may be used for triggering next stitch if gcode translation is to be used.

#### Credits:

---
2023-03-16
