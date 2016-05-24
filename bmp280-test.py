#!/usr/bin/python
# -*- coding: iso-8859-15 -*-
'''
Created on 23.05.2016

@author: micha
'''

import logging

from bmp280 import BMP280

if __name__ == "__main__":

    bmp = None

    try:
        bmp = BMP280()
        print 'temperature: ' + str(bmp.read_temperature())
        print 'pressure: ' + str(bmp.read_pressure()/100.0)
        print 'altitude: ' + str(bmp.read_altitude())
        print 'sealevel pressure: ' + str(bmp.read_sealevel_pressure(310.0)/100.0)
    except Exception, e:
        logging.critical(str(e))
