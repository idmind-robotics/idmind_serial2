#idmind_serial package
This package implements serial communication with different devices.

## Requirements
pyserial

## Usage
 - After running catkin_make, do `from idmind_serial.idmind_serialport import IDMindSerial`
 - Start the serial port with `IDMindSerial(addr, baudrate, timeout,  verify_checksum, verbose)`
 - Use `write_command(bytearray(msg))` and `read_command(nr_bytes)` for simple communication
 - Use `command(msg, nr_bytes)` for a robust communication  