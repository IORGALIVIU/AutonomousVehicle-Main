#!/usr/bin/env python3
"""
PCA9685 I2C Write/Read Test
Tests if registers can be written to
"""

import smbus2
import time

PCA9685_ADDR = 0x42
bus = smbus2.SMBus(1)

print("=" * 50)
print("PCA9685 I2C Write/Read Test")
print("=" * 50)

# Test 1: Write to MODE2
print("\n[Test 1] Writing 0x04 to MODE2 (register 0x01)...")
bus.write_byte_data(PCA9685_ADDR, 0x01, 0x04)
time.sleep(0.01)
result = bus.read_byte_data(PCA9685_ADDR, 0x01)
print(f"Read back: 0x{result:02X}")
if result == 0x04:
    print("✅ Write SUCCESS!")
else:
    print(f"❌ Write FAILED! Expected 0x04, got 0x{result:02X}")

# Test 2: Write to MODE1
print("\n[Test 2] Writing 0x20 to MODE1 (register 0x00)...")
bus.write_byte_data(PCA9685_ADDR, 0x00, 0x20)
time.sleep(0.01)
result = bus.read_byte_data(PCA9685_ADDR, 0x00)
print(f"Read back: 0x{result:02X}")
if result == 0x20:
    print("✅ Write SUCCESS!")
else:
    print(f"❌ Write FAILED! Expected 0x20, got 0x{result:02X}")

# Test 3: Multiple writes to MODE1
print("\n[Test 3] Testing multiple writes to MODE1...")
test_values = [0x00, 0x20, 0x10, 0x80]
for val in test_values:
    bus.write_byte_data(PCA9685_ADDR, 0x00, val)
    time.sleep(0.01)
    result = bus.read_byte_data(PCA9685_ADDR, 0x00)
    status = "✅" if result == val else "❌"
    print(f"  Write 0x{val:02X} → Read 0x{result:02X} {status}")

# Test 4: Read all important registers
print("\n[Test 4] Reading all important registers...")
registers = {
    0x00: "MODE1",
    0x01: "MODE2", 
    0xFE: "PRESCALE",
    0x06: "LED0_ON_L",
    0x07: "LED0_ON_H",
    0x08: "LED0_OFF_L",
    0x09: "LED0_OFF_H"
}

for addr, name in registers.items():
    val = bus.read_byte_data(PCA9685_ADDR, addr)
    print(f"  {name:12s} (0x{addr:02X}): 0x{val:02X} ({val:3d})")

bus.close()

print("\n" + "=" * 50)
print("DIAGNOSIS:")
print("If ALL writes show wrong values → Module defect or WP pin active")
print("If reads show 0x00 → Communication issue")
print("If reads show 0xFF → Module not responding properly")
print("=" * 50)
