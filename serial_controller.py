import serial
import threading
import time
import math
from enum import Enum
from typing import List, Optional, Tuple
from queue import Queue


class MessageType(Enum):
    """The types of messages for the pico"""
    MSG_PWM = 0x01
    MSG_MANUAL = 0x02
    MSG_CONFIG = 0x03
    MSG_RETURN_RX = 0x80
    MSG_RETURN_PID = 0x81
    MSG_RETURN_ADC = 0x82


class SerialController:
    """
    Handle serial communication with the microcontroller.
    Supports sending shot PWM pulses, manual commands, and configuration commands

    Protocol Format:
    [START_BYTE][TYPE][LENGTH_MSB][LENGTH_LSB][DATA...][CHECKSUM][END_BYTE]
    """

    START_BYTE = 0xAA
    END_BYTE = 0xFF
    MAX_PULSES = 16000 # Max pulses per packet - ensure agreement with the transmitter

    def __init__(self, port: str, baudrate: int = 9600, timeout: float = 1.0, debug=False):
        """
        Initialize the serial connection
        """
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout, bytesize=8, parity='N', stopbits=1)

        self.received_packets = []
        self._listener_running = False
        self._listener_thread = None
        self._response_queue = Queue()
        self._serial_lock = threading.Lock()
        self._debug_enabled = debug

        time.sleep(2) # Waits for connection to establish TODO: add check here

    def close(self):
        """Closes the serial connection and stops debug thread."""
        self.stop_listener_thread()
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
        checksum = self.calculate_checksum(data)

        return bytes(header + data + [checksum, self.END_BYTE])
    
    def parse_packet(self, packet: bytes) -> Optional[Tuple[MessageType, List[int]]]:
        """
        Parse a recieved packet and return (message_type, data) if valid
        Returns None if packt is invalid
        """

        if len(packet) < 6:
            return None
        
        if packet[0] != self.START_BYTE or packet[-1] != self.END_BYTE:
            return None
        
        # Checksum includes only data
        data_start = 4
        data_end = -2 # exclude checksum and end
        data = packet[data_start : data_end]
        calculated_checksum = 0
        for elem in data:
            calculated_checksum ^= elem
        if calculated_checksum != packet[-2]:
            print(f"checksum wrong: calc={calculated_checksum}, recv={packet[-2]}")
            return None
        
        try:
            msg_type = MessageType(packet[1])
        except ValueError:
            return None
        
        length = (packet[2] << 8) | packet[3]
        if (4+length)>(len(packet)-2): # Check if length matches
            return None #TODO: Error handling
        data = list(packet[4:4+length])

        return (msg_type, data)
    
    def start_listener_thread(self):
        """
        Start the serial listener thread
        """
        if self._listener_running:
            return
        
        self._listener_running = True
        self._listener_thread = threading.Thread(
            target=self._serial_listener,
            daemon=True
        )
        self._listener_thread.start()

    def stop_listener_thread(self):
        """
        Stop the serial listener thread
        """
        self._listener_running = False
        if self._listener_thread is not None:
            self._listener_thread.join()
            self._listener_thread = None

    def _serial_listener(self):
        """
        Single thread that handles all incoming seria data:
        - Protocol packets
        - Debug messages
        """
        buffer = bytearray()
        debug_prefixes = {b'[ERROR]', b'[WARN]', b'[INFO]', b'[DEBUG]'}

        while self._listener_running:
            try:
                with self._serial_lock:
                    if self.ser.in_waiting:
                        byte = self.ser.read(1)
                        buffer += byte
                        # print(f"raw: {buffer.hex()}")

                        # Check for protocol packet first
                        if len(buffer) >= 6 and buffer[0] == self.START_BYTE and buffer[-1] == self.END_BYTE: # TODO: Prevent buffer overflow
                            parsed = self.parse_packet(bytes(buffer))
                            if parsed:
                                self.received_packets.append(parsed)
                                if parsed[0] == MessageType.MSG_RETURN_RX or parsed[0] == MessageType.MSG_RETURN_ADC or parsed[0] == MessageType.MSG_RETURN_PID:
                                    self._response_queue.put(parsed)
                                buffer = bytearray() # clear buffer
                                continue

                        # Check for debug messages
                        elif len(buffer) > 0 and buffer[0] == 0x5B: # '['
                            if buffer[-1] == 0x0A: # '\n'
                                if self._debug_enabled:
                                    try:
                                        message = buffer.decode('utf-8').strip()
                                        if any(message.startswith(prefix.decode('utf-8')) for prefix in debug_prefixes):
                                            print(message)
                                        elif message:
                                            print(f"[SERIAL] {message}") # TODO: change to [UNKNOWN]?
                                    except UnicodeDecodeError:
                                        print(f"[BINARY] {buffer.hex()}")
                                
                                buffer = bytearray() # clear buffer

                    else:
                        time.sleep(0.01)

            except Exception as e:
                print(f"Error in serial listener: {e}")
                buffer = bytearray()
                        

    def read_packet(self, timeout: float = 1.0) -> Optional[Tuple[MessageType, List[int]]]:
        """
        Read and parse a single packet from the serial port
        Returns (message_type, data) if successful, None otherwise
        """
        start_time = time.time()
        buffer = bytearray()

        while time.time() - start_time < timeout:
            with self._serial_lock:
                if self.ser.in_waiting:
                    byte = self.ser.read(1)
                    buffer += byte

                    # Check for a complete packet
                    if len(buffer) >= 6 and buffer[0] == self.START_BYTE and buffer[-1] == self.END_BYTE:
                        parsed = self.parse_packet(bytes(buffer))
                        if parsed:
                            self.received_packets.append(parsed)
                            if parsed[0] == MessageType.MSG_RETURN_RX or parsed[0] == MessageType.MSG_RETURN_ADC or parsed[0] == MessageType.MSG_RETURN_PID:
                                self._response_queue.put(parsed)
                            
                            return parsed
                        buffer = bytearray() # Reset buffer if packet is invalid
                else:
                    time.sleep(0.01)

        return None
    

    def get_response(self, timeout:float = 1.0) -> Optional[Tuple[MessageType, List[int]]]:
        """
        Get the next response packet from the queue
        Returns None if no response received within timeout
        """
        try:
            return self._response_queue.get(timeout=timeout)
        except:
            return None

    def send_pwm_pulses(self, pulses: List[int]):
        """
        Send an array of PWM pulse values corresponding to one shot to the pico
        """
        
        if (len(pulses) > self.MAX_PULSES):
            print(f"Number of pulses ({len(pulses)}) is too long. Returning") # TODO: better error handling
            
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



if __name__ == "__main__":
    try:
        # Initialize controller (change port as needed)
        controller = SerialController(port='COM3', debug=True)
        controller.start_listener_thread()

        # Test usage
        maximum = 15000
        nextNum = 100
        flip = False
        pulseList = []
        for i in range(0, maximum):
            pulseList.append(int(math.floor(nextNum)))

            if flip == False:
                nextNum += 0.05
            elif flip == True:
                nextNum -= 0.05
            
            if nextNum >= 200:
                flip = True
            elif nextNum <= 1:
                flip = False
            # if i%2 == 0:
            #     pulseList.append(50)
            # else:
            #     pulseList.append(150)
        #print(f"sendingData={pulseList}")
        print("Sending PWM pulses...")
        controller.send_pwm_pulses(pulseList)

        for i in range(0, 5):
            response = controller.get_response(timeout=2.0)
            if response:
                msg_type, data = response
                print(f"Received response {i}: Type={msg_type}, Data={data}")
            else:
                print(f"none received {i}")


    except serial.SerialException as e:
        print(f"Serial error: {e}")

    finally:
        controller.close()
