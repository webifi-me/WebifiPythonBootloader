#!/usr/bin/python

"""
Webifi Ltd assumes no responsibility or liability for the use of the software.
This software is supplied "AS IS" without any warranties. Webifi Ltd reserves
the right to make changes in the software without notification. This software can be
freely distributed but must only be used with the service provided by Webifi Ltd.

Before running this program, update the details in settings.ini
"""

import os
import sys
import ConfigParser
import crcmod
import serial
import time
import math
import Queue
from threading import Thread


def calc_crc(message):
    poly = 0x1D0F
    reg=0
    for byte in message:
        mask = 0x80
        while(mask > 0):
            #left shift by one
            reg<<=1
            #input the next bit from the message into the right hand side of the op reg
            if byte & mask:
                reg += 1
            mask>>=1
            #if a one popped out the left of the reg, xor reg w/poly
            if reg > 0xffff:
                #eliminate any one that popped out the left
                reg &= 0xffff
                #xor with the poly, this is the remainder
                reg ^= poly
    reg &= 0xffff
    return reg


class SerialPort:
    def __init__(self, port):
        # configure serial port
        self._rx_callback = None
        self._parity = serial.PARITY_NONE
        self._baudrate = 115200
        self._number_of_bits = serial.EIGHTBITS
        self._stop_bits = serial.STOPBITS_ONE
        self._port = port
        self._serial = None
        self._running = False
        self._thread = None
        self._receive_state_header = 0
        self._receive_state_header2 = 1
        self._receive_state_data = 2
        self._header_size_len_crc = 6
        self._header_crc_offset = 4
        self._header_length_offset_msb = 0
        self._header_length_offset_lsb = 1
        self._rec_data_buf = bytearray()
        self._send_header = 'WBC'
        self._rec_header = 'WBR'
        self._data_len = 0
        self._rec_state = self._receive_state_header

    def set_rx_callback(self, rx_callback):
        self._rx_callback = rx_callback

    def open_serial_port(self):
        self._serial = serial.Serial(port=self._port, baudrate=self._baudrate, parity=self._parity,
                                     stopbits=self._stop_bits, bytesize=self._number_of_bits, timeout=0.25)
        # Create thread
        self._running = True
        self._thread = Thread(target=self._update)
        self._thread.start()

    def close_serial_port(self):
        self._running = False
        self._serial.close()

    def _check_header_crc(self):
        if len(self._rec_data_buf) != self._header_size_len_crc:
            return False   # received data is not correct to be a valid packet
        header_crc = self._rec_data_buf[self._header_crc_offset] << 8
        header_crc += self._rec_data_buf[self._header_crc_offset+1]
        # Calculate the header CRC
        crc_func = crcmod.mkCrcFun(0x11021, rev=False, initCrc=0x0000, xorOut=0x0000)
        crc = crc_func(self._rec_header + str(self._rec_data_buf[0:self._header_crc_offset]))
        if crc != header_crc:
            return False
        return True

    def _check_data_crc(self):
        data_len = self._rec_data_buf[0] << 8
        data_len += self._rec_data_buf[1]
        if len(self._rec_data_buf) - self._header_size_len_crc != self._data_len:
            return False        # received data is not the correct length
        data_crc = self._rec_data_buf[2] << 8
        data_crc += self._rec_data_buf[3]
        payload = self._rec_data_buf[self._header_size_len_crc:]
        crc_func = crcmod.mkCrcFun(0x11021, rev=False, initCrc=0x0000, xorOut=0x0000)
        crc = crc_func(str(payload))
        if crc != data_crc:
            return False
        return True

    def _update(self):
        while self._running:
            c = self._serial.read(1)
            if c:
                if self._rec_state == self._receive_state_header:
                    if c == self._rec_header[self._data_len].encode():
                        self._data_len += 1
                        if self._data_len >= len(self._rec_header):
                            self._rec_state = self._receive_state_header2
                            self._rec_data_buf = bytearray()
                    elif c == self._rec_header[0].encode():
                        self._data_len = 1
                    else:
                        self._data_len = 0
                elif self._rec_state == self._receive_state_header2:
                    self._rec_data_buf += c
                    if len(self._rec_data_buf) >= self._header_size_len_crc:
                        # All the length and CRC bytes has been received
                        if self._check_header_crc():
                            # Header has correct CRC
                            self._rec_state = self._receive_state_data
                            self._data_len = self._rec_data_buf[self._header_length_offset_msb] << 8
                            self._data_len |= self._rec_data_buf[self._header_length_offset_lsb]
                        else:
                            # Header CRC not correct, reset everything
                            self._rec_state = self._receive_state_header
                            self._data_len = 0
                elif self._rec_state == self._receive_state_data:
                    self._rec_data_buf += c
                    if len(self._rec_data_buf) - self._header_size_len_crc >= self._data_len:
                        # All the data has been received
                        if self._check_data_crc():
                            if self._rx_callback:
                                packet = self._rec_data_buf[self._header_size_len_crc:]
                                self._rx_callback(packet)
                        self._rec_state = self._receive_state_header
                        self._data_len = 0
        return False

    def send_data(self, data):
        if self._running:
            self._serial.write(data)

    def to_bytes(self, n, length, endianess='big'):
        h = '%x' % n
        s = ('0'*(len(h) % 2) + h).zfill(length*2).decode('hex')
        return s if endianess == 'big' else s[::-1]

    # add the header for a payload packet which can be sent to the bootloader
    def serial_protocol_build(self, payload):
        packet = bytes(self._send_header)
        packet += self.to_bytes(len(payload), 2)
        crc_func = crcmod.mkCrcFun(0x11021, rev=False, initCrc=0x0000, xorOut=0x0000)
        if type(payload) is str:
            crc = crc_func(bytes(payload))
        else:
            crc = crc_func(payload)
        packet += self.to_bytes(crc, 2)
        crc_func = crcmod.mkCrcFun(0x11021, rev=False, initCrc=0x0000, xorOut=0x0000)
        crc = crc_func(packet)
        packet += self.to_bytes(crc, 2)
        if type(payload) is str:
            packet += bytes(payload)
        else:
            packet += payload
        return packet

responseQueue = Queue.Queue()

# callback which will be called by serial port when data is received
def data_received(packet):
    global responseQueue
    responseQueue.put(packet)

if __name__ == "__main__":
    timeout_fail = 30  # *100ms
    if len(sys.argv) == 2:
        settings_filename = sys.argv[1]
    else:
        settings_filename = 'settings.ini'
    config = ConfigParser.ConfigParser()
    config.read(settings_filename)
    try:
        param_firmware_file = config.get("BootloaderDetails", "firmwareFile")
        param_serial_port = config.get("BootloaderDetails", "serialPort")
    except Exception as error:
        print(error)
        print("ERROR : Cannot parse input parameters from " + settings_filename)
        sys.exit(1)
    if not os.path.isfile(param_firmware_file):
        print("ERROR : "+param_firmware_file+" not found")
        sys.exit(1)
    # read the firmware binary
    total_bytes = os.path.getsize(param_firmware_file)
    with open(param_firmware_file, "rb") as input_file:
        input_firmware = input_file.read(total_bytes)
    serial_port = SerialPort(param_serial_port)
    try:
        serial_port.open_serial_port()
    except Exception as error:
        print(error)
        print('ERROR : Could not open serial port')
        sys.exit(1)
    serial_port.set_rx_callback(data_received)

    # first try to communicate with PCB and see if acknowledge is sent back
    packet = serial_port.serial_protocol_build('A')
    serial_port.send_data(packet)
    timeout = timeout_fail
    while timeout > 0:
        timeout -= 1
        time.sleep(0.1)
        if not responseQueue.empty():
            response = responseQueue.get()
            if len(response) == 1 and response[0] == ord('A'):
                print('Device connected')
                break
    if timeout == 0:
        print('ERROR : Could not connect with the device')
        serial_port.close_serial_port()
        sys.exit(1)

    # erase flash
    num_pages = total_bytes / 2048
    if total_bytes % 2048 > 0:
        num_pages += 1
    packet = bytes('E')
    packet += serial_port.to_bytes(num_pages, 1)
    packet = serial_port.serial_protocol_build(packet)
    serial_port.send_data(packet)
    timeout = timeout_fail
    while timeout > 0:
        timeout -= 1
        time.sleep(0.1)
        if not responseQueue.empty():
            response = responseQueue.get()
            if len(response) == 2:
                if response[0] == ord('E')and response[1] == ord('S'):
                    print('Flash erase started')
                elif response[0] == ord('E')and response[1] == ord('F'):
                    print('Flash erase completed')
                    break
    if timeout == 0:
        print('ERROR : Error erasing flash')
        serial_port.close_serial_port()
        sys.exit(1)

    # program flash
    programming = True
    send_packet = True
    current_packet = 0
    bytes_per_packet = 512
    total_packets = len(input_firmware) / bytes_per_packet
    if len(input_firmware) % bytes_per_packet > 0:
        total_packets += 1
    print('Programming flash')
    print_timeout = 20  # *100ms
    print_cnt = print_timeout
    while programming:
        if send_packet:
            send_packet = False
            timeout = timeout_fail
            index = current_packet * bytes_per_packet  # 512 bytes per packet
            indexEnd = index + bytes_per_packet
            if indexEnd > len(input_firmware):
                indexEnd = len(input_firmware)
            packet = bytes('F')
            packet += serial_port.to_bytes(current_packet, 2)
            last_packet = 1
            if current_packet < total_packets - 1:
                last_packet = 0
            packet += serial_port.to_bytes(last_packet, 1)
            packet += input_firmware[index:indexEnd]
            packet = serial_port.serial_protocol_build(packet)
            serial_port.send_data(packet)
            current_packet += 1
        timeout -= 1
        time.sleep(0.1)
        if timeout == 0:
            programming = False
        print_cnt -= 1
        if print_cnt == 0:
            print_cnt = print_timeout
            print('Programming progress: ' + str(round(current_packet*100/total_packets)) + '%')
        if not responseQueue.empty():
            response = responseQueue.get()
            if len(response) == 1:
                if response[0] == ord('P'):
                    if current_packet < total_packets:
                        send_packet = True
                    else:
                        programming = False
                if response[0] == ord('e'):
                    print('ERROR : Error programming flash')
                    serial_port.close_serial_port()
                    sys.exit(1)
    if timeout == 0:
        print('ERROR : Error programming flash')
        serial_port.close_serial_port()
        sys.exit(1)

    # waiting for verify
    timeout = timeout_fail
    while timeout > 0:
        timeout -= 1
        time.sleep(0.1)
        if not responseQueue.empty():
            response = responseQueue.get()
            if len(response) == 2:
                if response[0] == ord('V') and response[1] == ord('S'):
                    print("Verify started")
                elif response[0] == ord('V')and response[1] == ord('P'):
                    print('Verify passed')
                    print('Programming completed successfully')
                    print('Remove Settings jumper to run application')
                    break
                elif response[0] == ord('V')and response[1] == ord('E'):
                    print('Verify failed')
                    print('Programming failed')
                    break
    if timeout == 0:
        print('ERROR : Verify timed out')
        print('Programming failed')
        serial_port.close_serial_port()
        sys.exit(1)

    serial_port.close_serial_port()