#!/usr/bin/python
# -*- coding: iso-8859-15 -*-
'''
Created on 10.03.2015

@author: micha
'''

import logging
import serial
import MySQLdb as mdb

from util import formatData, description, sensor 

# the main procedure

logging.basicConfig(format='%(asctime)s\t%(levelname)s\t%(message)s', level=logging.INFO)
logging.info("Starting weather station sensor logging")

con = None
port = None

try:

    con = mdb.connect('localhost', 'davis', 'davis', 'davis');
    cur = con.cursor()

    port = serial.Serial(
        port='/dev/ttyUSB0',\
        baudrate=115200,\
        parity=serial.PARITY_NONE,\
        stopbits=serial.STOPBITS_ONE,\
        bytesize=serial.EIGHTBITS,\
        timeout=None)

    stage = 0;

    # main loop
    while True:
        line = port.readline().strip()

        if stage == 0 and line[0] == '?':
            stage = 1
        elif stage == 1 and line[0] == '#':
            port.write("x200\n") # Threshold set to -100db
            stage = 2
        elif stage == 2 and line[0] == '#':
            port.write("t1\n") # Tansmitter 1
            stage = 3
        elif stage == 3 and line[0] == '#':
            port.write("f1\n") # Filter on
            stage = 4
        elif stage == 4 and line[0] == '#':
            port.write("o0\n") # Output original data
            stage = 5
        elif stage == 5 and line[0] == '#':
            port.write("m1\n") # Frequency band 1
            stage = 6
        elif stage == 6 and len(line) > 3:
            sid = line[0]
            if sid == 'B' or sid == 'I' or sid == 'W' or sid == 'T' or sid == 'R' or sid == 'P':
                cur.execute("INSERT INTO logger(sensor,data,description) VALUES(%s,%s,%s)", (sensor(line),formatData(line),description(line)))
                con.commit()

except Exception, e:
    logging.critical(str(e))

finally:
    if con:
        con.close()
    if port:
        port.close()
