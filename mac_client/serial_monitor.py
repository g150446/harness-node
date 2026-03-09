#!/usr/bin/env python3
"""
XIAO Serial Monitor

Connects to XIAO boards over USB serial and displays log output.
Handles the XIAO nRF54L15 Sense CMSIS-DAP port more reliably than the
original ESP32-specific implementation.
"""

import sys
import signal
import time
import argparse
import subprocess
import re
import os

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
DEFAULT_TIMEOUT = 0.1
ANSI_ESCAPE_RE = re.compile(r"\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")
CONTROL_CHAR_RE = re.compile(r"[\x00-\x08\x0b-\x1f\x7f-\x9f]")
ORPHAN_COLOR_CODE_RE = re.compile(r"\[[0-9;]*m")


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


def sanitize_serial_line(line):
    """Strip ANSI escapes and non-printable control bytes from serial output."""
    clean_line = ANSI_ESCAPE_RE.sub("", line)
    clean_line = ORPHAN_COLOR_CODE_RE.sub("", clean_line)
    clean_line = CONTROL_CHAR_RE.sub("", clean_line)
    return clean_line


# ============================================================================
# Serial Port Functions
# ============================================================================

def detect_board_type(port_info):
    """Infer the board family from serial port metadata."""
    manufacturer = (port_info.manufacturer or "").lower()
    description = (port_info.description or "").lower()

    if "nrf54" in description or "cmsis-dap" in description:
        return "nrf54l15"
    if "esp32" in description or "usb jtag" in description:
        return "esp32s3"
    if "xiao" in manufacturer:
        return "xiao"
    return "unknown"


def list_serial_ports():
    """List available serial ports."""
    ports = serial.tools.list_ports.comports()
    xiao_ports = []
    other_ports = []
    
    for port in ports:
        manufacturer = port.manufacturer or ""
        description = port.description or ""
        board_type = detect_board_type(port)
        
        # XIAO ESP32S3 uses USB Serial/JTAG
        if board_type != "unknown" or 'usbmodem' in port.device or 'USB Serial' in description:
            xiao_ports.append(port)
        else:
            other_ports.append(port)
    
    return xiao_ports, other_ports


def find_xiao_port():
    """Find a likely XIAO serial port."""
    xiao_ports, _ = list_serial_ports()
    
    if xiao_ports:
        return xiao_ports[0].device
    
    # If no XIAO found, return first available port
    all_ports = serial.tools.list_ports.comports()
    if all_ports:
        return all_ports[0].device
    
    return None


def prefer_monitor_port(port):
    """Prefer the tty device for passive log monitoring on macOS."""
    if port.startswith("/dev/cu."):
        tty_port = "/dev/tty." + port[len("/dev/cu."):]
        if os.path.exists(tty_port):
            return tty_port
    return port


def print_ports(xiao_ports, other_ports):
    """Print available serial ports."""
    print("\n=== Available Serial Ports ===\n")
    
    if xiao_ports:
        print("XIAO devices:")
        for port in xiao_ports:
            board_type = detect_board_type(port)
            print(f"  {port.device}")
            print(f"    Description: {port.description}")
            print(f"    Manufacturer: {port.manufacturer}")
            print(f"    Board type: {board_type}")
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
    """Serial monitor for XIAO boards."""
    
    def __init__(self, port, baud_rate=DEFAULT_BAUD_RATE, timeout=DEFAULT_TIMEOUT):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_conn = None
        self.line_buffer = ""
        self.board_type = self._detect_connected_board_type()
        self.drop_partial_line = True

    def _detect_connected_board_type(self):
        candidates = {self.port}
        if self.port.startswith("/dev/tty."):
            candidates.add("/dev/cu." + self.port[len("/dev/tty."):])
        elif self.port.startswith("/dev/cu."):
            candidates.add("/dev/tty." + self.port[len("/dev/cu."):])
        for port_info in serial.tools.list_ports.comports():
            if port_info.device in candidates:
                return detect_board_type(port_info)
        return "unknown"
        
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
            time.sleep(0.5)
            self.drop_partial_line = True
            print(f"Connected to {self.port} at {self.baud_rate} baud ({self.board_type})")
            return True
        except serial.SerialException as e:
            print(f"Error opening port {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from serial port."""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("Disconnected")
        self.serial_conn = None
    
    def reconnect(self, wait_seconds=4.0):
        """Reconnect after a device-side reset or USB reconnect."""
        deadline = time.time() + wait_seconds
        self.disconnect()
        self.line_buffer = ""
        while running and time.time() < deadline:
            if self.connect():
                return True
            time.sleep(0.2)
        return False
    
    def reset_device(self):
        """Reset the connected board."""
        print("Resetting device...")

        if self.board_type == "nrf54l15":
            try:
                subprocess.run(
                    ["pyocd", "reset", "-t", "nrf54l"],
                    check=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                if not self.reconnect(wait_seconds=6.0):
                    print("Warning: device did not reconnect after pyocd reset")
                return
            except (FileNotFoundError, subprocess.CalledProcessError):
                print("Warning: pyocd reset failed, falling back to serial control")

        if self.serial_conn and self.serial_conn.is_open:
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
            if self.drop_partial_line and '\n' in self.line_buffer:
                _, self.line_buffer = self.line_buffer.split('\n', 1)
                self.drop_partial_line = False

            if '\n' in self.line_buffer:
                line, self.line_buffer = self.line_buffer.split('\n', 1)
                return sanitize_serial_line(line)

            if self.serial_conn.in_waiting:
                try:
                    chunk_size = max(1, self.serial_conn.in_waiting)
                    chunk = self.serial_conn.read(chunk_size).decode('utf-8', errors='ignore')
                    if chunk:
                        self.line_buffer += chunk
                except serial.SerialException as e:
                    print(f"Read error: {e}")
                    if not self.reconnect():
                        return None
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
        description='XIAO Serial Monitor',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                      # Auto-detect XIAO and connect
  %(prog)s -p /dev/tty.usbmodem1101  # Specify port
  %(prog)s -b 74880             # Use different baud rate
  %(prog)s --list               # List available ports
  %(prog)s --auto-reset         # Reset before monitoring
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
        '--auto-reset',
        action='store_true',
        help='Force auto-reset on connect'
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
    monitor_port = prefer_monitor_port(port)
    if monitor_port != port:
        print(f"Using monitor port: {monitor_port}")
        port = monitor_port
    
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
    print("XIAO Serial Monitor")
    print("=" * 60)
    print()
    
    default_auto_reset = monitor.board_type not in {"nrf54l15", "esp32s3"}
    if monitor.board_type == "esp32s3" and not args.auto_reset and not args.no_auto_reset:
        print("Note: auto-reset is disabled by default for ESP32-S3 USB JTAG boards to avoid entering download mode.")
    auto_reset = default_auto_reset
    if args.auto_reset:
        auto_reset = True
    if args.no_auto_reset:
        auto_reset = False
    success = monitor.run(auto_reset=auto_reset)
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
