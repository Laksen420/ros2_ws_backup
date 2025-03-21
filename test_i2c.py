#!/usr/bin/env python3

import board
import busio
import time
import sys

def main():
    print("Starting I2C test")
    try:
        # Initialize I2C
        i2c = busio.I2C(board.SCL, board.SDA)
        print("I2C initialized")
        
        # Wait for I2C to stabilize
        time.sleep(0.5)
        
        # Scan for I2C devices
        print("Scanning I2C bus...")
        devices = i2c.scan()
        if devices:
            print(f"Found {len(devices)} I2C devices at addresses:")
            for device in devices:
                print(f"  0x{device:02x}")
        else:
            print("No I2C devices found!")
            
        # Try to lock the bus
        print("Attempting to lock I2C bus...")
        if i2c.try_lock():
            print("Bus locked successfully!")
            i2c.unlock()
            print("Bus unlocked")
        else:
            print("Failed to lock I2C bus!")
            
        # Try to read from the IMU addresses
        for addr in [0x4A, 0x4B]:
            print(f"Testing communication with device at 0x{addr:02X}...")
            try:
                if i2c.try_lock():
                    try:
                        # Try to read a byte from the device
                        buffer = bytearray(1)
                        i2c.readfrom_into(addr, buffer)
                        print(f"Successfully read from 0x{addr:02X}: {buffer[0]}")
                    except Exception as e:
                        print(f"Error reading from 0x{addr:02X}: {e}")
                    finally:
                        i2c.unlock()
                else:
                    print("Could not lock I2C bus!")
            except Exception as e:
                print(f"Exception while communicating with 0x{addr:02X}: {e}")
                
    except Exception as e:
        print(f"Error initializing I2C: {e}")
        
    print("I2C test complete")

if __name__ == "__main__":
    main()
