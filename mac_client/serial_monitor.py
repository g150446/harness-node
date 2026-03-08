#!/usr/bin/env python3
"""
XIAO ESP32S3 Serial Monitor

Connects to XIAO ESP32S3 Sense via USB serial and displays log output.
"""

import sys
import signal
import time
import argparse

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Please install pyserial: pip install pyserial")
    sys.exit(1)


# ============================================================================
# Configuration
# ============================================================================

DEFAULT_BAUD_RATE = 115200
DEFAULT_TIMEOUT = 0.01


# ============================================================================
# Signal Handler
# ============================================================================

running = True

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    global running
    running = False
    print("\n\nExiting...")
    sys.exit(0)


# ============================================================================
# Serial Port Functions
# ============================================================================

def list_serial_ports():
    """List available serial ports."""
    ports = serial.tools.list_ports.comports()
    xiao_ports = []
    other_ports = []
    
    for port in ports:
        manufacturer = port.manufacturer or ""
        description = port.description or ""
        
        # XIAO ESP32S3 uses USB Serial/JTAG
        if 'XIAO' in manufacturer or 'ESP32' in description or 'USB JTAG' in description:
            xiao_ports.append(port)
        elif 'usbmodem' in port.device or 'USB Serial' in description:
            xiao_ports.append(port)
        else:
            other_ports.append(port)
    
    return xiao_ports, other_ports


def find_xiao_port():
    """Find XIAO ESP32S3 serial port."""
    xiao_ports, _ = list_serial_ports()
    
    if xiao_ports:
        return xiao_ports[0].device
    
    # If no XIAO found, return first available port
    all_ports = serial.tools.list_ports.comports()
    if all_ports:
        return all_ports[0].device
    
    return None


def print_ports(xiao_ports, other_ports):
    """Print available serial ports."""
    print("\n=== Available Serial Ports ===\n")
    
    if xiao_ports:
        print("XIAO ESP32S3 / ESP devices:")
        for port in xiao_ports:
            print(f"  {port.device}")
            print(f"    Description: {port.description}")
            print(f"    Manufacturer: {port.manufacturer}")
            print()
    
    if other_ports:
        print("Other devices:")
        for port in other_ports:
            print(f"  {port.device}")
            print(f"    Description: {port.description}")
            print()
    
    if not xiao_ports and not other_ports:
        print("No serial ports found.")


# ============================================================================
# Serial Monitor
# ============================================================================

class SerialMonitor:
    """Serial monitor for XIAO ESP32S3."""
    
    def __init__(self, port, baud_rate=DEFAULT_BAUD_RATE, timeout=DEFAULT_TIMEOUT):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_conn = None
        self.line_buffer = ""
        
    def connect(self):
        """Connect to serial port."""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
            )
            # Wait for the connection to establish
            time.sleep(0.5)
            print(f"Connected to {self.port} at {self.baud_rate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error opening port {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from serial port."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Disconnected")
    
    def reset_device(self):
        """Reset XIAO by toggling DTR/RTS."""
        if self.serial_conn and self.serial_conn.is_open:
            print("Resetting device...")
            self.serial_conn.setDTR(False)
            self.serial_conn.setRTS(True)
            time.sleep(0.1)
            self.serial_conn.setRTS(False)
            time.sleep(0.5)
    
    def read_line(self):
        """Read a line from serial."""
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
        
        while running:
            if self.serial_conn.in_waiting:
                try:
                    char = self.serial_conn.read().decode('utf-8', errors='ignore')
                    if char:
                        self.line_buffer += char
                        if char == '\n':
                            line = self.line_buffer
                            self.line_buffer = ""
                            return line
                except Exception as e:
                    print(f"Read error: {e}")
                    return None
            time.sleep(0.001)
        
        return None
    
    def run(self, auto_reset=True):
        """Run the serial monitor."""
        if not self.connect():
            return False
        
        if auto_reset:
            self.reset_device()
        
        print("Monitoring serial output (Press Ctrl+C to stop)...\n")
        print("-" * 60)
        
        try:
            while running:
                line = self.read_line()
                if line:
                    # Filter and display ESP-IDF log lines
                    if line.strip():
                        print(line.rstrip())
        except KeyboardInterrupt:
            pass
        finally:
            self.disconnect()
        
        return True


# ============================================================================
# Main Entry Point
# ============================================================================

def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='XIAO ESP32S3 Serial Monitor',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                      # Auto-detect XIAO and connect
  %(prog)s -p /dev/tty.usbmodem1101  # Specify port
  %(prog)s -b 74880             # Use different baud rate
  %(prog)s --list               # List available ports
  %(prog)s --reset              # Reset device and exit
        """
    )
    
    parser.add_argument(
        '-p', '--port',
        type=str,
        help='Serial port (default: auto-detect)'
    )
    parser.add_argument(
        '-b', '--baud',
        type=int,
        default=DEFAULT_BAUD_RATE,
        help=f'Baud rate (default: {DEFAULT_BAUD_RATE})'
    )
    parser.add_argument(
        '-t', '--timeout',
        type=float,
        default=DEFAULT_TIMEOUT,
        help=f'Read timeout in seconds (default: {DEFAULT_TIMEOUT})'
    )
    parser.add_argument(
        '--list',
        action='store_true',
        help='List available serial ports'
    )
    parser.add_argument(
        '--reset',
        action='store_true',
        help='Reset device and exit'
    )
    parser.add_argument(
        '--no-auto-reset',
        action='store_true',
        help='Disable auto-reset on connect'
    )
    
    args = parser.parse_args()
    
    # Set up signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # List ports
    if args.list:
        xiao_ports, other_ports = list_serial_ports()
        print_ports(xiao_ports, other_ports)
        return 0
    
    # Find port
    port = args.port
    if not port:
        port = find_xiao_port()
        if not port:
            print("Error: No serial port found. Use --list to see available ports.")
            return 1
        print(f"Auto-detected port: {port}")
    
    # Create monitor
    monitor = SerialMonitor(port, args.baud, args.timeout)
    
    # Reset only
    if args.reset:
        if monitor.connect():
            monitor.reset_device()
            monitor.disconnect()
        return 0
    
    # Run monitor
    print("=" * 60)
    print("XIAO ESP32S3 Serial Monitor")
    print("=" * 60)
    print()
    
    auto_reset = not args.no_auto_reset
    success = monitor.run(auto_reset=auto_reset)
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
