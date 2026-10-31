"""Log the M340 controller state over Modbus TCP into a CSV file.

Every 0.1 s the script reads 16 holding registers starting at %MW100 and
decodes eight 32-bit REAL values. Stop logging with Ctrl+C.

Register map (two words per REAL, low word first):

    %MW100  SetPoint      speed setpoint [RPM]
    %MW102  RPM_Stator    stator speed from the drive
    %MW104  PWM           controller output (drive command)
    %MW106  Kp            current gains from the DD_IPD function block
    %MW108  Ki
    %MW110  Kd
    %MW112  ScanTime_ms   execution time measured on the PLC [ms]
    %MW114  RPM           measured speed [RPM]

Usage (from the repository root):

    python m340/tools/modbus_logger.py initial_run.csv
    python m340/tools/modbus_logger.py dd_efrit_run.csv --host 10.102.57.131
"""
import argparse
import csv
import struct
import sys
import time

from pymodbus.client import ModbusTcpClient


def read_real(regs, low):
    """Decode a Schneider 32-bit REAL: word order little, byte order big."""
    return struct.unpack('>f', struct.pack('>HH', regs[low + 1], regs[low]))[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("output_file", help="CSV file to create")
    parser.add_argument("--host", default="10.102.57.131", help="PLC IP address")
    parser.add_argument("--port", type=int, default=502)
    args = parser.parse_args()

    client = ModbusTcpClient(args.host, port=args.port)
    if not client.connect():
        print("Failed to connect")
        sys.exit(1)

    with open(args.output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['SetPoint', 'RPM', 'PWM', 'Kp', 'Ki', 'Kd', 'ScanTime_ms', 'RPM_Stator'])

        try:
            while True:
                result = client.read_holding_registers(address=100, count=16)

                if not result.isError():
                    regs = result.registers
                    speed_sp = read_real(regs, 0)
                    rpm_stator = read_real(regs, 2)
                    pwm_out = read_real(regs, 4)
                    kp_val = read_real(regs, 6)
                    ki_val = read_real(regs, 8)
                    kd_val = read_real(regs, 10)
                    scan_time = read_real(regs, 12)
                    rpm_val = read_real(regs, 14)

                    writer.writerow([speed_sp, rpm_val, pwm_out, kp_val, ki_val, kd_val, scan_time, rpm_stator])
                    print(f"SP: {speed_sp:.1f} | RPM: {rpm_val:.1f} | PWM: {pwm_out:.1f} | P:{kp_val:.4f} I:{ki_val:.4f} D:{kd_val:.4f} | Scan:{scan_time:.1f}ms | RPM_Stator:{rpm_stator:.4f}")
                else:
                    print("Error: Could not read Modbus registers.")

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\nLogging stopped by user.")
        finally:
            client.close()
            print(f"Connection closed. Data saved to {args.output_file}")


if __name__ == "__main__":
    main()
