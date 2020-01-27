#!/usr/bin/env python

import time
import serial
from threading import Lock


class IDMindSerial(serial.Serial):
    """Driver to connect to serial devices (imu, boards...)"""

    def __init__(self, addr, baudrate=115200, timeout=2, verify_checksum=False, verbose=0):
        """
        Initiates the serial connection with the device in address addr, using a specified baudrate.
        :param addr:
        :param baudrate:
        :param timeout:
        """
        self.verbose = verbose
        self.verify_checksum = verify_checksum
        self.mutex = Lock()

        try:
            serial.Serial.__init__(self, port=addr, baudrate=baudrate, timeout=timeout)
            if self.verbose > 5:
                print "Connection to " + addr + " was successful"
        except serial.SerialException as e:
            if self.verbose > 5:
                print("Connection to "+addr+" failed with: " + str(e))
            raise serial.SerialException(e)

    @staticmethod
    def to_bytes(val, nr_bytes=2):
        """
        NEW: Transforms an integer into the requested number of bytes
        OLD: Transforms an integer [-32767, 32767] to 2 bytes
        :param val:
        :param nr_bytes: number of bytes to convert to
        :return [higher_byte, ...lower_byte]:
        """
        val = int(val)
        msg = []
        for i in range(nr_bytes, 0, -1):
            msg.append((val >> 8*(i-1)) & 0xFF)
        return msg

    @staticmethod
    def to_num(b_array, unsigned=False):
        """
        Transforms a bytearray to an integer
        :param b_array:     Bytearray
        :param unsigned:    If true, returns unsigned [0, 65535], else returns in range [-32767, 32767]
        :return res:        Integer
        """
        byte_nr = len(b_array)
        res = 0
        for idx in range(0, byte_nr):
            res = res | (ord(b_array[idx]) << 8*(byte_nr-idx-1))
        if res > pow(2, 8*byte_nr)/2 and not unsigned:
            res = -(pow(2, 8*byte_nr)-res+1)
        return res

    def command(self, msg, nr_bytes, tries=5):
        """

        :param msg:
        :param nr_bytes:
        :param tries:
        :return:
        """

        # Check if the port is open
        if not self.is_open:
            raise serial.SerialException(1, "Serial port is not open")

        # Convert message to bytearray
        try:
            len(msg)
        except TypeError:
            msg = bytearray([msg])
        else:
            msg = bytearray(msg)

        # Write message to serial port and wait for response. Raise Exception in case of failure
        try:
            self.mutex.acquire()
            b = self.send_command(msg=msg, tries=tries)
            if b == 0:
                raise serial.SerialException(2, "Unable to send command")
            elif b != len(msg):
                raise serial.SerialException(3, "Failed to send complete message")
            else:
                if self.verbose > 8:
                    print "Message {} send".format(msg)

            res = self.read_command(nr_bytes=nr_bytes, tries=tries)
            if res == 0:
                raise serial.SerialException(4, "No response from device")
            elif res < nr_bytes:
                raise serial.SerialException(4, "Incomplete response from device")
            else:
                if self.verify_checksum:
                    checksum = self.to_num(res[-2], res[-1])
                    bytesum = reduce(lambda x, y: x + y, [ord(el) for el in res[:-2]])
                    if ord(res[0]) == msg[0] and checksum == (bytesum & 0xffff):
                        return res
                    else:
                        serial.SerialException(4, "Checksum error")
                else:
                    return res
        except Exception as e:
            raise e
        finally:
            self.mutex.release()

    def send_command(self, msg, tries=5):
        """
        Sends a message through the serial port. The message should be a bytearray.
        Returns the number of bytes written to the port.
        :param msg:
        :param tries:
        :return written_bytes:
        """
        t = 0
        while t < tries:
            try:
                self.reset_input_buffer()
                self.reset_output_buffer()
                res = self.write(msg)
                return res
            except serial.SerialException as e:
                if self.verbose:
                    print e
                t = t + 1
                time.sleep(0.01)

        return 0

    def read_command(self, nr_bytes, tries=5):
        """
        Reads a specified number of bytes from the port and returns as a list
        :param nr_bytes:
        :param tries:
        :return [byte_1, ... byte_nr_bytes]:
        """
        res = []
        t = 0
        while t < tries:
            try:
                res = self.read(nr_bytes)
                return res
            except serial.SerialException as e:
                if self.verbose:
                    print e
                t = t + 1
                time.sleep(0.01)
        return res

    def restart_port(self):
        try:
            serial.Serial.__init__(self, port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            if self.verbose > 5:
                print "Connection to " + self.port + " was successful"
        except serial.SerialException as e:
            if self.verbose > 5:
                print("Connection to "+self.port+" failed with: " + str(e))
            raise serial.SerialException(e)


if __name__ == ":_main__":
    s = IDMindSerial("/dev/ttyACM0")
