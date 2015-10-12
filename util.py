# -*- coding: iso-8859-15 -*-
'''
Created on 12.03.2015

@author: buchfink
'''

import math

POLYNOMIAL = 0x1021
PRESET = 0

def _initial(c):
    crc = 0
    c = c << 8
    for _ in range(8):
        if (crc ^ c) & 0x8000:
            crc = (crc << 1) ^ POLYNOMIAL
        else:
            crc = crc << 1
        c = c << 1
    return crc

_tab = [ _initial(i) for i in range(256) ]

def _update_crc(crc, c):
    cc = 0xff & c

    tmp = (crc >> 8) ^ cc
    crc = (crc << 8) ^ _tab[tmp & 0xff]
    crc = crc & 0xffff

    return crc

# Calculates the crc
def crc(data):
    crc = PRESET
    for idx in range(2, 10):
        crc = _update_crc(crc, int(data[idx], 16))
    return crc

# formats the line
def formatData(line):
    data = line.split()
    if len(data) == 0 or data[0] != 'I':
        return line

    value = ""
    for idx in range(len(data)):
        if idx > 0:
            value += " "
        if idx >= 2 and idx < 10:
            value += ("0" + data[idx]) if len(data[idx]) == 1 else data[idx]
        elif idx == 10:
            value += (" " + data[idx])
        else:
            value += data[idx]
    return value

# returns the sensor id
def sensor(line):
    if len(line) == 0:
        return ' '
    elif line[0] <> 'I':
        return line[0]
    elif len(line) > 6:
        if line[6] == '2':
            return 'V'
        elif line[6] == '5':
            return 'R'
        elif line[6] == '7':
            return 'S'
        elif line[6] == '8':
            return 'T'
        elif line[6] == '9':
            return 'G'
        elif line[6] == 'A':
            return 'H'
        elif line[6] == 'E':
            return 'N'
        else:
            return 'I'
    else:
        return 'I'

# Parses values from line
def description(line):
    description = "unknown"
    data = line.split()

    # Pressure
    if data[0] == 'B' and len(data) == 7:
        p = round(math.pow(math.pow(float(data[4])/100.0, 0.1902614) + 8.417168e-05 * 310.5, 5.255927), 1)
        description = "p=" + str(p) + "hPa"

    elif data[0] == 'I' and crc(data) != 0:
        return "invalid crc"

    elif data[0] == 'I':
        header = int(data[2], 16) >> 4
        windSpeed = int(round(int(data[3], 16) * 1.609344, 0))
        windDirections = (int(data[4], 16) << 2) | (int(data[6], 16) & 0x02)
        windDirections = 360 if windDirections > 1024 or windDirections <= 0 else int(round(windDirections * 360.0 / 1024.0))  
        windData = " w(" + str(windSpeed) + "km/h, " + str(windDirections) + "°)"

        # akku voltage
        if header == 0x2:
            voltage = round(((int(data[5], 16) << 2) + ((int(data[6], 16) & 0xC0) >> 6)) / 100.0, 1)
            description = "v=" + str(voltage) + "V"

        elif header == 0x3:
            description = "unknown" # not implemented

        # rain rate
        elif header == 0x5:
            rr = 0
            rr1 = int(data[5], 16)
            rr2 = int(data[6], 16)
            if rr1 != 0xff:
                if (rr2 & 0x40) == 0:
                    rr = round(11520.0/(((rr2 & 0x30) << 4) | rr1), 1)
                elif (rr2 & 0x40) == 0x40:
                    rr = round(11520.0/(((rr2 & 0x30) << 8) | (rr1 << 4)), 1)
            description = "rr=" + str(rr) + "mm/h"

        # solar radiation
        elif header == 0x7:
            sol = (int(data[5], 16) << 2) + ((int(data[6], 16) & 0xC0) >> 6)
            description = "sol=" + str(sol)

        # temperature
        elif header == 0x8:
            value = int(data[5], 16) * 256 + int(data[6], 16)
            value = value - 65536 if value > 32767 else value
            temperature = round((value/160.0 - 32.0)*5.0/9.0, 1)
            description = "t=" + str(temperature) + "°C"

        # gust speed
        elif header == 0x9:
            gustSpeed = int(round(int(data[5], 16) * 1.609344, 0))
            description = "g=" + str(gustSpeed) + "km/h"

        # humidity
        elif header == 0xA:
            value = ((int(data[6], 16) >> 4) << 8) + int(data[5], 16)
            humidity = int(round(value * 1.01 / 10.0, 0))
            humidity = 100 if humidity > 100 else humidity
            description = "h=" + str(humidity) + "%"

        # rain ticks
        elif header == 0xE:
            ticks = value = int(data[5], 16) & 0x7f
            description = "r=" + str(ticks)

        description = "%-12s%s" % (description,windData)

    return description
