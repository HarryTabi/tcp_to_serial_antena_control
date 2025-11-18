"""
TCP to Serial Antenna Control Bridge for GPredict

This script bridges GPredict (satellite tracking software) to antenna rotator 
controllers via serial communication. It receives tracking commands over TCP
and translates them to serial commands for azimuth and elevation control.

Original Author: Antoine Scherer (ISU IT Intern)
Original Repository: https://github.com/antscher/tcp_to_serial_antena_control

Modifications by: Harry Tabi Ndip (Leanspace)
- Fixed master/slave initialization order (azimuth before elevation)
- Enhanced serial reader error handling with SerialException catching
- Added sequential controller initialization for hardware stability
- Improved error messages and logging
- Added CPU usage optimization with sleep delays in readers
- Changed serial ports from Windows COM to Linux /dev/ttyUSB
"""

import socket
import time
import threading
import serial

# TCP server parameters for Gpredict
TCP_HOST = "localhost"
TCP_PORT = 4533

# Serial parameters
SERIAL_AZ_PORT = "/dev/ttyUSB0"   # Azimuth controller (Master)
SERIAL_EL_PORT = "/dev/ttyUSB1"   # Elevation controller (Slave)
SERIAL_BAUD = 9600

# Shared current positions
current_az = 0.0
current_el = 0.0

# Locks for thread-safe variable access
lock = threading.Lock()


def serial_reader_az(ser):
    """
    Thread function to read AZIMUTH feedback with better error handling.
    
    Continuously reads azimuth position feedback from the controller.
    Handles serial errors gracefully and prevents high CPU usage.
    
    Args:
        ser: Serial port object for azimuth controller
    """
    global current_az
    
    while True:
        try:
            # The timeout on the serial port (set during initialization) will
            # prevent readline() from blocking indefinitely.
            line = ser.readline().decode(errors='ignore').strip()
            if line and line.startswith("A="):
                parts = line.split()
                az = float(parts[0][2:])
                with lock:
                    current_az = az
        except serial.SerialException as e:
            print(f"[SERIAL AZ] Serial Error: {e}")
            break  # Exit thread on error
        except (ValueError, IndexError) as e:
            print(f"[SERIAL AZ] ERROR parsing azimuth: {line} | Exception: {e}")
        time.sleep(0.1)  # Small delay to prevent high CPU usage


def serial_reader_el(ser):
    """
    Thread function to read ELEVATION feedback with better error handling.
    
    Continuously reads elevation position feedback from the controller.
    Handles serial errors gracefully and prevents high CPU usage.
    
    Args:
        ser: Serial port object for elevation controller
    """
    global current_el
    
    while True:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if line and line.startswith("E="):
                parts = line.split()
                el = float(parts[0][2:])
                with lock:
                    current_el = el
        except serial.SerialException as e:
            print(f"[SERIAL EL] Serial Error: {e}")
            break  # Exit thread on error
        except (ValueError, IndexError) as e:
            print(f"[SERIAL EL] ERROR parsing elevation: {line} | Exception: {e}")
        time.sleep(0.1)  # Small delay to prevent high CPU usage


def tcp_server(ser_az, ser_el):
    """
    TCP server compatible with Gpredict.
    
    Handles position requests and movement commands for AZ and EL.
    Implements the GPredict Rotctld protocol for antenna control.
    
    Commands:
        p - Query current position (returns "az\\nel\\n")
        P <az> <el> - Set new position
        S - Stop
        q - Quit
    
    Args:
        ser_az: Serial port object for azimuth controller
        ser_el: Serial port object for elevation controller
    """
    global current_az, current_el

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((TCP_HOST, TCP_PORT))
    server_socket.listen(1)

    print(f"Waiting for Gpredict connection on {TCP_HOST}:{TCP_PORT}...")

    while True:
        conn, addr = server_socket.accept()
        print(f"Connected to {addr}")

        while True:
            data = conn.recv(1024)
            if not data:
                break  # Client disconnected

            command = data.decode().strip()
            print(f"[TCP] Command received: {command}")

            if command == "p":
                # Return current position
                with lock:
                    response = f"{current_az:.1f}\n{current_el:.1f}\n"
                conn.sendall(response.encode())
                print(f"[TCP] Position sent: {response.strip()}")

            elif command.startswith("P "):
                # Move to new position: P <az> <el>
                parts = command.split()
                if len(parts) == 3:
                    az = float(parts[1].replace(",", "."))
                    el = float(parts[2].replace(",", "."))

                    # Send commands to azimuth and elevation controllers
                    cmd_az = f"A{az:.1f}\r"
                    cmd_el = f"E{el:.1f}\r"

                    ser_az.write(cmd_az.encode())
                    time.sleep(0.5) 
                    ser_el.write(cmd_el.encode())

                    print(f"[AZIMUTH SERIAL] Command sent: {cmd_az.strip()}")
                    print(f"[ELEVATION SERIAL] Command sent: {cmd_el.strip()}")

                    # Return updated positions (instantaneous)
                    with lock:
                        response = f"{current_az:.1f}\n{current_el:.1f}\n"
                    conn.sendall(response.encode())
                    print(f"[TCP] Position sent: {response.strip()}")

            if command == "S" or command == "q":
                break  # Exit the loop to stop the server 

        conn.close()
        print("[TCP] Client disconnected")


def main():
    """
    Main function to initialize serial ports sequentially for master/slave hardware,
    start serial reading threads, and run the TCP server.
    
    IMPORTANT: Initializes Azimuth (Master) before Elevation (Slave) to ensure
    proper hardware initialization and prevent communication errors.
    
    This sequential initialization fixes stability issues that occurred when
    both controllers were initialized simultaneously.
    """
    print("[INFO] Initializing controllers in Master/Slave order...")

    # Initialize Azimuth (Master) first - CRITICAL for stability
    print("[INFO] Opening Azimuth port...")
    ser_az = serial.Serial(SERIAL_AZ_PORT, SERIAL_BAUD, timeout=1)
    time.sleep(2)  # Wait for the master controller to be ready
    print("[INFO] Azimuth port opened.")

    # Initialize Elevation (Slave) second
    print("[INFO] Opening Elevation port...")
    ser_el = serial.Serial(SERIAL_EL_PORT, SERIAL_BAUD, timeout=1)
    time.sleep(2)  # Wait for the slave controller to be ready
    print("[INFO] Elevation port opened.")

    # Launch reading threads for azimuth and elevation
    threading.Thread(target=serial_reader_az, args=(ser_az,), daemon=True).start()
    threading.Thread(target=serial_reader_el, args=(ser_el,), daemon=True).start()

    # Start TCP server loop
    tcp_server(ser_az, ser_el)


if __name__ == "__main__":
    main()
