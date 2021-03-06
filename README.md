![Library in Use](../assets/images/BB_I2C.jpg)
BB_I2C library - Bit-banged I2C master interface over arbitrary digital pins 
for the Arduino Zero.

Copyright (c) 2015 J.Bordens

Derrived from a blog post found at the following URL:
<http://codinglab.blogspot.com/2008/10/i2c-on-avr-using-bit-banging.html>
by Raul.  GPLv3 license listed in the comments of this blog post.

License & Disclaimer
--------------------
This program is licensed under the GPL v3 - see LICENSE.txt for more info.

BECAUSE THE PROGRAM IS LICENSED FREE OF CHARGE, THERE IS NO WARRANTY FOR THE 
PROGRAM, TO THE EXTENT PERMITTED BY APPLICABLE LAW. EXCEPT WHEN OTHERWISE 
STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR OTHER PARTIES PROVIDE THE 
PROGRAM "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, 
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK AS TO THE QUALITY AND 
PERFORMANCE OF THE PROGRAM IS WITH YOU. SHOULD THE PROGRAM PROVE DEFECTIVE, 
YOU ASSUME THE COST OF ALL NECESSARY SERVICING, REPAIR OR CORRECTION.

IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN WRITING WILL 
ANY COPYRIGHT HOLDER, OR ANY OTHER PARTY WHO MAY MODIFY AND/OR REDISTRIBUTE 
THE PROGRAM AS PERMITTED ABOVE, BE LIABLE TO YOU FOR DAMAGES, INCLUDING ANY
GENERAL, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF THE USE
OR INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF DATA OR 
DATA BEING RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU OR THIRD PARTIES OR
A FAILURE OF THE PROGRAM TO OPERATE WITH ANY OTHER PROGRAMS), EVEN IF SUCH 
HOLDER OR OTHER PARTY HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.


Notes
-----
This is a very quick-and-dirty implementation of a bit-banged I2C interface.
It has been tested with a SSD1306 display (Write-only) and a BMP085 pressure 
sensor (read/write) on the same bus.

By default this library uses A4/A5, similar to old shields which had their
I2C interface on these pins.  This doesn't mean you should try using old
shields with this library because the Zero is *NOT* 5v compatible.

Change Log
----------
1.0.0 - Initial release