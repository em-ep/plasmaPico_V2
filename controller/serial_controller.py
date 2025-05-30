import serial
import time
from enum import Enum
from typing import List, Union


class MessageType(Enum):
    """The types of messages for the pico"""
    MSG_PWM = 0x01
    MSG_MANUAL = 0x02
    MSG_CONFIG = 0x03


class SerialController:
    """
    Handle serial communication with the microcontroller.
    Supports sending shot PWM pulses, manual commands, and configuration commands

    Protocol Format:
    [START_BYTE][TYPE][LENGTH_MSB][LENGTH_LSB][DATA...][CHECKSUM][END_BYTE]
    """

    START_BYTE = 0xAA
    END_BYTE = 0x55
    MAX_PULSES = 16000 # Max pulses per packet - ensure agreement with the transmitter

    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 1.0):
        """
        Initialize the serial connection
        """
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout, bytesize=8, parity='N', stopbits=1)
        time.sleep(2) # Waits for connection to establish TODO: add check here

    def close(self):
        """Closes the serial connection."""
        self.ser.close()

    def calculate_checksum(self, data: List[int]) -> int:
        """Calculate XOR checksum for the given data"""
        checksum = 0

        for byte in data:
            checksum ^= byte
        
        return checksum
    

    def build_packet(self, msg_type: MessageType, data: List[int]) -> bytes:
        """
        Construct a complete packet 

        Protocol Format:
        [START_BYTE][TYPE][LENGTH_MSB][LENGTH_LSB][DATA...][CHECKSUM][END_BYTE]
        """

        length = len(data)
        if length > self.MAX_PULSES:
            raise ValueError(f"Data too long ({length} bytes), max is {self.MAX_PULSES}")
        
        length_msb = (length >> 8) & 0xFF
        length_lsb = length & 0xFF

        header = [self.START_BYTE, msg_type.value, length_msb, length_lsb]
        checksum = self.calculate_checksum(header + data)

        return bytes(header + data + [checksum, self.END_BYTE])
    

    def send_pwm_pulses(self, pulses: List[int]):
        """
        Send an array of PWM pulse values corresponding to one shot to the pico
        """
        
        if (len(pulses) > self.MAX_PULSES):
            print(f"Number of puses ({len(pulses)}) is too long. Returning") # TODO: better error handling
            
            return -1

        packet = self.build_packet(MessageType.MSG_PWM, pulses)

        #print(' '.join(f'{x:02x}' for x in packet))  

        self.ser.write(packet)

        print(f"Sent {len(pulses)} PWM pulses")

    
    def send_manual_control(self, switchConfig: List[int]): # TODO: see if bools work here for serial communication
        """
        Sends manual switch configuration command

        Args:
            switchConfig: [S1, S2, S3, S4] states (0 or 1) in list
        """

        packet = self.build_packet(MessageType.MSG_MANUAL, switchConfig)
        self.ser.write(packet)

        print(f"Sent manual commang: {switchConfig}")

    
    def send_configuration(self, param_id: int, value: int):
        """
        Not yet implimented
        """

controller = SerialController(port='COM3')

pulseList = []

for i in range(0, 100):
    if i%2 == 0:
        pulseList.append(50)
    else:
        pulseList.append(150)

print(controller.ser.readline())
print(controller.ser.readline())
print(controller.ser.readline())

print("sending now")
controller.send_pwm_pulses(pulseList)

while True:
    print(controller.ser.readline())

time.sleep(10)


# if __name__ == "__main__":
#     try:
#         # Initialize controller (change port as needed)
#         controller = SerialController(port='COM3')

#     except serial.SerialException as e:
#         print(f"Serial error: {e}")


controller.close()