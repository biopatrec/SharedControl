# Background
This is the firmware code for the custom rangefinder board used on the
Erle-Copter. The board has a SHARP IR rangefinders located on the bottom
of the quadcopter, and has a range of between 10-80 cm.

# Building
The project was originally built for an ATTINY85 using the
[PlatformIO IDE](http://platformio.org/)

# Interface
The sensor board uses an **I2C** interface (with integrated 4.7k pull-up
resistors). Communication has been tested with up to a **5kHz** clock.

The board address is `0x66`, meaning writes are performed on address `0xCC`
and reads are performed on address `0xCD`.

**All** values are read-only, so writes are ignored. Reading from the device
causes it to send 1 byte representing the integer range in centimeters
detected by the sensor. Anything exceeding the range is set to 0xFF.

# Hardware
The system uses a
[GP2Y0A21YK0F](http://www.socle-tech.com/doc/IC%20Channel%20Product/Sensors/Distance%20Measuring%20Sensor/Analog%20Output/gp2y0a21yk_e.pdf)
Sharp IR analog sensors for distance measurements.
