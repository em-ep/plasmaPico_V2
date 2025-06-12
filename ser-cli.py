import argparse
import cmd
import math
import sys
from typing import List, Optional

from serial_controller import *

class SerialControllerCLI(cmd.Cmd):
    """Interactive command-line interface for SerialController"""
    
    intro = "Serial Controller CLI. Type 'help' for available commands."
    prompt = "> "
    
    def __init__(self, controller, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.controller = controller
        
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
    
    def do_send_pwm(self, arg):
        """
        Send PWM pulses to the device.
        Usage: send_pwm [options]
        
        Options:
          test <max_pulses> - Send a test pattern (default: 1000 pulses)
          file <filename>  - Send pulses from a file (one value per line)
          values <v1 v2...> - Send specific pulse values
        """
        args = arg.split()
        if not args:
            print("Error: Please specify a pulse source (test/file/values)")
            return
            
        if args[0] == "test":
            max_pulses = 1000
            if len(args) > 1:
                try:
                    max_pulses = int(args[1])
                except ValueError:
                    print("Error: max_pulses must be an integer")
                    return
                    
            pulse_list = self._generate_test_pulses(max_pulses)
            self.controller.send_pwm_pulses(pulse_list)
            
        # TODO: Add more presets and support for text files
                
        else:
            print("Error: Unknown pulse source. Use test/file/values")
            
    def _generate_test_pulses(self, max_pulses: int) -> List[int]:
        """Generate a test pulse pattern"""
        pulse_list = []
        next_num = 100
        flip = False
        
        for _ in range(max_pulses):
            pulse_list.append(int(math.floor(next_num)))
            
            if not flip:
                next_num += 0.05
            else:
                next_num -= 0.05
                
            if next_num >= 200:
                flip = True
            elif next_num <= 1:
                flip = False
                
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
            print(f"Received response: Type={msg_type}, Data={data}")
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

def list_ports():
    """List available serial ports"""
    import serial.tools.list_ports
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found")
    else:
        print("Available serial ports:")
        for port in ports:
            print(f"  {port.device} - {port.description}")

def execute_command(controller, args):
    """Execute a single command from argparse"""
    try:
        if args.command == "send_pwm":
            if args.test:
                pulse_list = generate_test_pulses(args.test)
                controller.send_pwm_pulses(pulse_list)
            elif args.file:
                with open(args.file, 'r') as f:
                    pulse_list = [int(line.strip()) for line in f if line.strip()]
                controller.send_pwm_pulses(pulse_list)
            elif args.values:
                pulse_list = [int(x) for x in args.values]
                controller.send_pwm_pulses(pulse_list)
                
        elif args.command == "manual":
            if len(args.switches) != 4:
                raise ValueError("Need exactly 4 switch values (0 or 1)")
            if any(s not in (0, 1) for s in args.switches):
                raise ValueError("Switch values must be 0 or 1")
            controller.send_manual_control(args.switches)
            
        elif args.command == "read":
            response = controller.get_response(timeout=args.timeout)
            if response:
                msg_type, data = response
                print(f"Received response: Type={msg_type}, Data={data}")
            else:
                print("No response received within timeout period")
                
        elif args.command == "interactive":
            SerialControllerCLI(controller).cmdloop()
            
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

def generate_test_pulses(max_pulses: int) -> List[int]:
    """Generate a test pulse pattern"""
    pulse_list = []
    next_num = 100
    flip = False
    
    for _ in range(max_pulses):
        pulse_list.append(int(math.floor(next_num)))
        
        if not flip:
            next_num += 0.05
        else:
            next_num -= 0.05
            
        if next_num >= 200:
            flip = True
        elif next_num <= 1:
            flip = False
            
    return pulse_list

def main():
    parser = argparse.ArgumentParser(
        description="Serial Controller CLI",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  %(prog)s COM3 interactive
  %(prog)s COM4 send_pwm --test 500
  %(prog)s COM5 manual --switches 1 0 1 0
  %(prog)s COM6 read --timeout 2.0
""")

    # Global arguments
    parser.add_argument("port", help="Serial port (use 'list' to show available ports)")
    parser.add_argument("--baudrate", type=int, default=9600, help="Baud rate (default: 9600)")
    parser.add_argument("--debug", action="store_true", help="Enable debug output")

    # Subcommands
    subparsers = parser.add_subparsers(dest="command", help="Command to execute", required=True)

    # Interactive mode
    parser_interactive = subparsers.add_parser("interactive", help="Start interactive mode")

    # Send PWM command
    parser_pwm = subparsers.add_parser("send_pwm", help="Send PWM pulses")
    pwm_group = parser_pwm.add_mutually_exclusive_group(required=True)
    pwm_group.add_argument("--test", type=int, metavar="PULSES", help="Send test pattern with given number of pulses")
    pwm_group.add_argument("--file", metavar="FILENAME", help="Send pulses from file")
    pwm_group.add_argument("--values", type=int, nargs="+", metavar="VALUE", help="Send specific pulse values")

    # Manual control command
    parser_manual = subparsers.add_parser("manual", help="Send manual switch configuration")
    parser_manual.add_argument("--switches", type=int, choices=[0, 1], nargs=4, 
                             metavar=("S1", "S2", "S3", "S4"), 
                             required=True, help="Switch states (0 or 1)")

    # Read command
    parser_read = subparsers.add_parser("read", help="Read response from device")
    parser_read.add_argument("--timeout", type=float, default=1.0, help="Timeout in seconds (default: 1.0)")

    # List ports command
    parser_list = subparsers.add_parser("list", help="List available serial ports")

    args = parser.parse_args()

    if args.command == "list":
        list_ports()
        sys.exit(0)

    if args.port == "list":
        list_ports()
        sys.exit(0)

    try:
        controller = SerialController(port=args.port, baudrate=args.baudrate, debug=args.debug)
        controller.start_listener_thread()
        
        if args.command != "interactive":
            execute_command(controller, args)
        else:
            SerialControllerCLI(controller).cmdloop()
            
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
    finally:
        if 'controller' in locals():
            controller.close()

if __name__ == "__main__":
    main()