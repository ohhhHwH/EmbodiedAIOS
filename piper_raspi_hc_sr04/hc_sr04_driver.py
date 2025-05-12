#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 wheatfox
import RPi.GPIO as gpio
import time
import signal
import sys
import yaml
import os
from typing import Dict, Optional, List


class HCSR04Driver:
    def __init__(self, trig_pin: int, echo_pin: int):
        """
        Initialize the HC-SR04 ultrasonic sensor driver

        Args:
            trig_pin (int): GPIO pin number for trigger
            echo_pin (int): GPIO pin number for echo
        """
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin

        gpio.setmode(gpio.BCM)
        gpio.setup(self.trig_pin, gpio.OUT)
        gpio.setup(self.echo_pin, gpio.IN)

        gpio.output(self.trig_pin, False)
        time.sleep(0.5)

    def get_distance(self) -> Optional[float]:
        """
        Get the distance measurement from the sensor

        Returns:
            float: Distance in centimeters, or None if measurement failed
        """
        try:
            gpio.output(self.trig_pin, False)
            time.sleep(0.1)
            gpio.output(self.trig_pin, True)
            time.sleep(0.00001)
            gpio.output(self.trig_pin, False)

            while gpio.input(self.echo_pin) == 0:
                pulse_start = time.time()

            while gpio.input(self.echo_pin) == 1:
                pulse_end = time.time()

            pulse_duration = pulse_end - pulse_start

            if pulse_duration >= 0.01746:
                return None

            distance = pulse_duration * 17000

            if distance > 300 or distance == 0:
                return None

            return round(distance, 3)

        except Exception as e:
            print(f"error getting distance: {e}")
            return None


class MultiHCSR04Driver:
    def __init__(self, sensor_configs: Dict[str, Dict[str, int]] = None):
        """
        Initialize multiple HC-SR04 ultrasonic sensors

        Args:
            sensor_configs (Dict[str, Dict[str, int]], optional): Dictionary of sensor configurations.
                If None, will try to load from sensors.yaml in the same directory.
                Example: {
                    'sensor1': {'trig': 27, 'echo': 17},
                    'sensor2': {'trig': 22, 'echo': 23},
                    'sensor3': {'trig': 24, 'echo': 25},
                    'sensor4': {'trig': 8, 'echo': 7},
                    ...
                }
        """
        if sensor_configs is None:
            # Try to load from YAML file
            current_dir = os.path.dirname(os.path.abspath(__file__))
            config_path = os.path.join(current_dir, "sensors.yaml")
            try:
                with open(config_path, "r") as f:
                    config = yaml.safe_load(f)
                    sensor_configs = config["sensors"]
                    print(
                        f"successfully loaded sensor configurations from {config_path}"
                    )
            except Exception as e:
                print(f"failed to load sensor configurations: {str(e)}")
                raise

        self.sensors = {}
        for name, config in sensor_configs.items():
            self.sensors[name] = HCSR04Driver(config["trig"], config["echo"])

    def get_all_distances(self) -> Dict[str, Optional[float]]:
        """
        Get distance measurements from all sensors

        Returns:
            Dict[str, Optional[float]]: Dictionary of sensor names and their distances
        """
        return {name: sensor.get_distance() for name, sensor in self.sensors.items()}

    def get_sensor_names(self) -> List[str]:
        """
        Get list of all sensor names

        Returns:
            List[str]: List of sensor names
        """
        return list(self.sensors.keys())

    def cleanup(self):
        """Clean up GPIO resources"""
        gpio.cleanup()


def main():
    def signal_handler(signal, frame):
        print("You pressed Ctrl+C!")
        driver.cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    # Initialize driver without explicit configs - will load from YAML
    driver = MultiHCSR04Driver()

    print("starting ultrasonic sensor driver")
    print(
        f"initialized {len(driver.get_sensor_names())} sensors: {', '.join(driver.get_sensor_names())}"
    )

    try:
        while True:
            distances = driver.get_all_distances()
            for sensor_name, distance in distances.items():
                if distance is None:
                    print(f"{sensor_name}: measurement failed")
                else:
                    print(f"{sensor_name}: {distance} cm")
            time.sleep(0.1)

    except Exception as e:
        print(f"error: {e}")
    finally:
        driver.cleanup()


if __name__ == "__main__":
    main()
