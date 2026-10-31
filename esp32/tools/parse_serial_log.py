"""Extract the DATA lines of an ESP32 serial log into a CSV file.

The firmware prints one line per sample:

    DATA,<setpoint>,<rpm>,<pwm>,<kp>,<ki>,<kd>

Every other line (plotter output, timing, status messages) is ignored.

Usage (from the repository root):

    python esp32/tools/parse_serial_log.py <serial_log> <output.csv>

    e.g. python esp32/tools/parse_serial_log.py data/esp32/serial_logs/trial6_initial.log initial_run.csv
"""
import argparse
import csv
import re

pattern = re.compile(r"DATA,([\d\s\.,-]+)")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("input_file", help="serial log written by the PlatformIO monitor")
    parser.add_argument("output_file", help="CSV file to create")
    args = parser.parse_args()

    print(f"Decoding file: {args.input_file}...")

    rows_found = []

    with open(args.input_file, 'r', encoding='utf-8', errors='ignore') as infile:
        for line in infile:
            match = pattern.search(line)
            if match:
                raw_numbers = match.group(1).strip()
                parts = [p.strip() for p in raw_numbers.split(',')]

                if len(parts) >= 6:
                    rows_found.append(parts[:6])

    if rows_found:
        with open(args.output_file, 'w', newline='', encoding='utf-8') as outfile:
            writer = csv.writer(outfile)
            writer.writerow(['SetPoint', 'RPM', 'PWM', 'Kp', 'Ki', 'Kd'])
            writer.writerows(rows_found)
        print(f"SUCCESS! Extracted {len(rows_found)} rows into {args.output_file}")
    else:
        print("Failed to find DATA. Check if the pattern in the log matches 'DATA,'")


if __name__ == "__main__":
    main()
