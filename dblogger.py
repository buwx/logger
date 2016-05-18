#!/usr/bin/python
# -*- coding: iso-8859-15 -*-
'''
Created on 01.04.2016

@author: micha
'''

import argparse
import logging
import math
import threading
import time
import pymysql as mdb

import RPi.GPIO as GPIO
import Adafruit_BMP.BMP085 as BMP085

from davisreceiver import DavisReceiver
from util import check, sensor, description

PRESSURE_COUNT = 7  # number of pressure measurements
LOOP_TIME = 60      # time between pressure messages in seconds
LED_PIN = 11        # the GPIO-PIN (GPIO17) of the flashing led
HEIGHT = 309.3      # the height of the sensor

class DataLogger(object):
    def __init__(self):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(LED_PIN, GPIO.OUT)

        self.con = mdb.connect('localhost', 'davis', 'davis', 'davis', charset='latin1');
        self.cur = self.con.cursor()
        self.bmp_sensor = BMP085.BMP085(mode=BMP085.BMP085_ULTRAHIGHRES)
        self.receiver = DavisReceiver()
        self.receiver.set_handler(self.process_message)
        self.lock = threading.Lock()

    def loop(self):
        self.receiver.calibration()
        self.receiver.receive_begin()

        # main loop
        while True:
            ts = int(time.time())
            self.sleep(ts + LOOP_TIME - ts % LOOP_TIME)

            temperature = self.bmp_sensor.read_temperature()
            pressure = 0.0;
            for _ in range(PRESSURE_COUNT):
                pressure += self.bmp_sensor.read_pressure()
            pressure /= PRESSURE_COUNT

            height = HEIGHT
            sea_level_pressure = math.pow(math.pow(pressure/100.0, 0.1902614) + 8.417168e-05 * height, 5.255927)

            lost_messages = self.receiver.lost_messages
            valid_messages = self.receiver.valid_messages
            self.receiver.valid_messages = 0
            self.receiver.lost_messages = 0

            message = "A " + str(int(round(temperature * 10, 0)))
            message += " " + str(int(round(pressure, 0)))
            message += " " + str(int(round(height * 100, 0)))
            message += " " + str(int(round(sea_level_pressure * 100, 0)))
            message += " " + str(valid_messages)
            message += " " + str(lost_messages)
            logging.debug(message + " " + description(message))
            self.store_message(message, True)

    def sleep(self, target):
        while True:
            ts = int(time.time())
            if ts < target:
                time.sleep(target - ts)
            else:
                break
            
    def shutdown(self):
        if self.con:
            self.con.commit()
            self.con.close()
        if self.receiver:
            self.receiver.shutdown()

    def process_message(self, message):
        if not check(message):
            logging.warn("invalid message received: " + message + " " + description(message))
            return False

        GPIO.output(LED_PIN, True)
        logging.debug(message + " " + description(message))
        self.store_message(message, False)
	time.sleep(0.1)
        GPIO.output(LED_PIN, False)
        return True

    def store_message(self, message, commit):
        with self.lock:
            ts = int(time.time()*1000)
            self.cur.execute("INSERT INTO sensor(dateTime,sensor,data,description) VALUES(%s,%s,%s,%s)", (ts,sensor(message),message,description(message)))
            if commit:
                self.con.commit()

# the main procedure

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--debug", help="more log output", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(format='%(asctime)s\t%(levelname)s\t%(message)s', level=logging.DEBUG if args.debug else logging.INFO)
    logging.info("Starting Davis-ISS logging")

    data_logger = None

    try:
        data_logger = DataLogger()
        data_logger.loop()
    except Exception, e:
        logging.critical(str(e))

    finally:
        logging.info("Terminating Davis-ISS logging")
        if data_logger:
            data_logger.shutdown()
