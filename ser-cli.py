import cmd
import math
import sys
from typing import List

from serial_controller import *


class SerialControllerCLI(cmd.Cmd):
    """Command-line interface for SerialController"""
    
    intro = "Serial Controller CLI. Type 'help' for available commands."
    prompt = "> "
    
    def __init__(self, port: str, baudrate: int = 9600, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.controller = SerialController(port=port, baudrate=baudrate, debug=True)
        self.controller.start_listener_thread()
        self.pulseData = []
        
    def preloop(self):
        """Initialize before command loop starts"""
        print(f"Connected to {self.controller.ser.port} at {self.controller.ser.baudrate} baud")
        
    def postloop(self):
        """Cleanup after command loop ends"""
        self.controller.close()
        print("Connection closed.")
        
    def do_exit(self, arg):
        """Exit the CLI"""
        print("Goodbye!")
        return True
    
    def do_quit(self, arg):
        """Exit the CLI"""
        return self.do_exit(arg)
    
    def do_send_pwm(self, arg):
        """
        Send PWM pulses to the device.
        Usage: send_pwm [options]
        
        Options:
          test <max_pulses> - Send a test pattern (default: 15000 pulses)
          file <filename>  - Send pulses from a file (one value per line)
          values <v1 v2...> - Send specific pulse values
        """
        args = arg.split()
        if not args:
            print("Error: Please specify a pulse source (test/file/values)")
            return
            
        tests = ['saw', 'sine', 'alt']
        if args[0] in tests:
            max_pulses = 15000
            if len(args) > 1:
                try:
                    max_pulses = int(args[1])
                except ValueError:
                    print("Error: max_pulses must be an integer")
                    return
                
            if args[0] == "saw":
                pulse_list = self._generate_saw_pulses(max_pulses)
            if args[0] == "sine":
                pulse_list = self._generate_sine_pulses(max_pulses)
            if args[0] == "alt":
                pulse_list = self._generate_alt_pulses(max_pulses)

            self.controller.send_pwm_pulses(pulse_list)
            
        elif args[0] == "file":
            if len(args) < 2:
                print("Error: Please specify a filename")
                return
            try:
                with open(args[1], 'r') as f:
                    pulse_list = [int(line.strip()) for line in f if line.strip()]
                self.controller.send_pwm_pulses(pulse_list)
            except Exception as e:
                print(f"Error reading file: {e}")
                
        elif args[0] == "values":
            try:
                pulse_list = [int(x) for x in args[1:]]
                self.controller.send_pwm_pulses(pulse_list)
            except ValueError:
                print("Error: All pulse values must be integers")
                
        else:
            print("Error: Unknown pulse source. Use test/file/values")
            
    
    def _generate_saw_pulses(self, max_pulses: int) -> List[int]:
        """Generate a test sawtooth pulse pattern"""
        pulse_list = []
        next_num = 100
        flip = True

        for _ in range(max_pulses):
            pulse_list.append(int(math.floor(next_num)))

            if not flip:
                next_num += 0.5
            else:
                next_num -= 0.5

            if next_num >= 200:
                flip = True
            elif next_num <= 1:
                flip = False

        return pulse_list
    
    def _generate_sine_pulses(self, max_pulses: int) -> List[int]:
        """Generate a test sine wave pulse pattern"""
        pulse_list = []
        periods = 2

        for i in range(max_pulses):
            pulse_list.append(int(math.floor(((100 * math.sin(math.radians(360 * (i/max_pulses) * periods))) + 100))))
        
        return pulse_list
    
    def _generate_alt_pulses(self, max_pulses: int) -> List[int]:
        """Generate an alternating pattern of pos and neg pulses"""
        pulse_list = []

        for i in range(max_pulses):
            if i%2 == 0:
                pulseList.append(50)
            else:
                pulseList.append(150)

        return pulse_list
        
    def do_manual(self, arg):
        """
        Send manual switch configuration.
        Usage: manual <s1> <s2> <s3> <s4>
        Each switch value should be 0 (off) or 1 (on).
        """
        try:
            switches = [int(x) for x in arg.split()]
            if len(switches) != 4:
                print("Error: Need exactly 4 switch values (0 or 1)")
                return
            if any(s not in (0, 1) for s in switches):
                print("Error: Switch values must be 0 or 1")
                return
                
            self.controller.send_manual_control(switches)
        except ValueError:
            print("Error: Switch values must be integers (0 or 1)")
            
    def do_read(self, arg):
        """
        Read responses from the device.
        Usage: read [timeout]
        timeout: Optional timeout in seconds (default: 1.0)
        """
        timeout = 1.0
        if arg:
            try:
                timeout = float(arg)
            except ValueError:
                print("Error: Timeout must be a number")
                return
                
        response = self.controller.get_response(timeout=timeout)
        if response:
            msg_type, data = response
            print(f"Received response: Type={msg_type}, Length={len(data)}")
        else:
            print("No response received within timeout period")
            
    def do_config(self, arg):
        """
        Send configuration command (not yet implemented)
        Usage: config <param_id> <value>
        """
        print("Configuration command not yet implemented")
            
    def do_ports(self, arg):
        """List available serial ports"""
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        if not ports:
            print("No serial ports found")
        else:
            print("Available serial ports:")
            for port in ports:
                print(f"  {port.device} - {port.description}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python cli.py <serial_port> [baudrate]")
        print("Available ports:")
        import serial.tools.list_ports
        ports = serial.tools.list_ports.comports()
        for port in ports:
            print(f"  {port.device} - {port.description}")
        sys.exit(1)
        
    port = sys.argv[1]
    baudrate = 9600
    if len(sys.argv) > 2:
        try:
            baudrate = int(sys.argv[2])
        except ValueError:
            print("Error: Baudrate must be an integer")
            sys.exit(1)
            
    try:
        SerialControllerCLI(port=port, baudrate=baudrate).cmdloop()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)