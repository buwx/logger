#!/usr/bin/python
# -*- coding: utf-8 -*-
'''
Created on 30.03.2019

@author: M.Buchfink (buwx.de)
'''

import argparse
import logging
import math
import threading
import time

import RPi.GPIO as GPIO
import spidev

IRQ_PIN = 22    # gpio irg pin
CH_ID = 1       # channel id

SENSIVITY = 140

ISS_FREQUENCIES = [14222256, 14226191, 14230129, 14224227, 14228161] # frequency settings
ISS_CHANNELS = len(ISS_FREQUENCIES)

TIMER_INTERVAL = (40.0 + CH_ID)/16
MAX_HOPS = 8

# RFM69 register names
REG_FIFO          = 0x00
REG_OPMODE        = 0x01
REG_DATAMODUL     = 0x02
REG_BITRATEMSB    = 0x03
REG_BITRATELSB    = 0x04
REG_FDEVMSB       = 0x05
REG_FDEVLSB       = 0x06
REG_FRFMSB        = 0x07
REG_FRFMID        = 0x08
REG_FRFLSB        = 0x09
REG_OSC1          = 0x0A
REG_RXBW          = 0x19
REG_AFCBW         = 0x1A
REG_AFCFEI        = 0x1E
REG_FEIMSB        = 0x21
REG_FEILSB        = 0x22
REG_RSSIVALUE     = 0x24
REG_DIOMAPPING1   = 0x25
REG_IRQFLAGS1     = 0x27
REG_IRQFLAGS2     = 0x28
REG_RSSITHRESH    = 0x29
REG_PREAMBLELSB   = 0x2D
REG_SYNCCONFIG    = 0x2E
REG_SYNCVALUE1    = 0x2F
REG_SYNCVALUE2    = 0x30
REG_PACKETCONFIG1 = 0x37
REG_PAYLOADLENGTH = 0x38
REG_FIFOTHRESH    = 0x3C
REG_PACKETCONFIG2 = 0x3D
REG_TESTLNA       = 0x58
REG_TESTDAGC      = 0x6F

# RFM69 register values
RF_OPMODE_SLEEP       = 0x00
RF_OPMODE_STANDBY     = 0x04
RF_OPMODE_SYNTHESIZER = 0x08
RF_OPMODE_TRANSMITTER = 0x0C
RF_OPMODE_RECEIVER    = 0x10

RF_DATAMODUL_MODULATIONSHAPING_10 = 0x02

RF_BITRATEMSB_19200 = 0x06
RF_BITRATELSB_19200 = 0x83

RF_FDEVMSB_4800 = 0x00
RF_FDEVLSB_4800	= 0x4e

RF_RXBW_DCCFREQ_010 = 0x40
RF_RXBW_MANT_20     = 0x08
RF_RXBW_EXP_4       = 0x04

RF_RXBW_DCCFREQ_010 = 0x40
RF_RXBW_MANT_20     = 0x08
RF_RXBW_EXP_3       = 0x03

RF_OSC1_RCCAL_DONE  = 0x40
RF_OSC1_RCCAL_START = 0x80

RF_AFCFEI_AFCAUTOCLEAR_ON = 0x08
RF_AFCFEI_AFCAUTO_ON      = 0x04

RF_DIOMAPPING1_DIO0_01 = 0x40 # PayloadReady
RF_DIOMAPPING1_DIO0_11 = 0xC0 # Rssi

RF_IRQFLAGS1_MODEREADY    = 0x80
RF_IRQFLAGS2_PAYLOADREADY = 0x04
RF_IRQFLAGS2_FIFOOVERRUN  = 0x10

RF_SYNC_ON            = 0x80
RF_SYNC_FIFOFILL_AUTO = 0x00
RF_SYNC_SIZE_2        = 0x08
RF_SYNC_TOL_0         = 0x00

RF_SYNC_BYTE1_VALUE   = 0xCB
RF_SYNC_BYTE2_VALUE   = 0x89

RF_PACKET1_FORMAT_FIXED      = 0x00
RF_PACKET1_DCFREE_OFF        = 0x00
RF_PACKET1_CRC_OFF           = 0x00
RF_PACKET1_CRCAUTOCLEAR_OFF  = 0x08
RF_PACKET1_ADRSFILTERING_OFF = 0x00

RF_PACKET2_RXRESTARTDELAY_2BITS = 0x10
RF_PACKET2_AUTORXRESTART_ON     = 0x02
RF_PACKET2_AES_OFF              = 0x00

RF69_MODE_SLEEP   = 0
RF69_MODE_STANDBY = 1
RF69_MODE_SYNTH	  = 2
RF69_MODE_RX      = 3
RF69_MODE_TX	  = 4

def reverse_bits(b):
    b = ((b & 0b11110000) >>4 ) | ((b & 0b00001111) << 4)
    b = ((b & 0b11001100) >>2 ) | ((b & 0b00110011) << 2)
    b = ((b & 0b10101010) >>1 ) | ((b & 0b01010101) << 1)
    return b

def msb(b):
    return (b & 0xff0000) >> 16

def mid(b):
    return (b & 0x00ff00) >> 8

def lsb(b):
    return b & 0x0000ff

class Timer(threading.Thread):
    def __init__(self, delay, callback):
        threading.Thread.__init__(self)
        self.delay = delay
        self.callback = callback
        self.condition = threading.Condition()
        self.active = True
        self.calibrated = False
        self.timestamp = 0
        self.freq_index = 0
        self.daemon = True

    def run(self):
        with self.condition:
            while self.active:
                self.wait()
                self.do_callback()

    def wait(self):
        delay = self.delay
        if self.timestamp > 0:
            current = time.time()
            n = int((current - self.timestamp) / self.delay)
            if self.calibrated:
                self.calibrated = False
                n = 0.25
            else:
                n += 1.75
            delay = self.timestamp + n * self.delay - current
            if delay < 0.05:
                delay = 0.05

        logging.debug("wait " + str(delay))
        self.condition.wait(delay)

    def do_callback(self):
        if self.callback and not self.calibrated:
            current = time.time()
            frac = (1 + self.freq_index + (current - self.timestamp) / self.delay) % ISS_CHANNELS
            self.callback(int(frac))

    def cancel(self):
        with self.condition:
            self.active = False
            self.callback = None
            self.condition.notify()
        self.join()

    def calibrate(self, callback, timestamp, freq_index):
        self.callback = callback
        self.timestamp = timestamp
        self.freq_index = freq_index
        self.calibrated = True
        logging.debug("notify")
        self.condition.notify()

class DavisReceiver(object):
    def __init__(self):
        self.mode = None
        self.handler = None
        self.freq_index = 0
        self.valid_messages = 0
        self.lost_messages = 0
        self.hop_count = 1
        self.lock = False
        self.fei_array = [0, 0, 0, 0, 0]

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(IRQ_PIN, GPIO.IN)

        self.CONFIG = {
          0x01: [REG_OPMODE, RF_OPMODE_STANDBY], # Standby
          0x02: [REG_DATAMODUL, RF_DATAMODUL_MODULATIONSHAPING_10], # Packet Mode, FSK, Gaussian Filter, BT = 0.5
          # Davis uses a datarate of 19.2 KBPS
          0x03: [REG_BITRATEMSB, RF_BITRATEMSB_19200],
          0x04: [REG_BITRATELSB, RF_BITRATELSB_19200],
          # Davis uses a deviation of 4.8 kHz on Rx
          0x05: [REG_FDEVMSB, RF_FDEVMSB_4800],
          0x06: [REG_FDEVLSB, RF_FDEVLSB_4800],
          # set channel frequency
          0x07: [REG_FRFMSB, msb(ISS_FREQUENCIES[self.freq_index] + self.fei_array[self.freq_index])],
          0x08: [REG_FRFMID, mid(ISS_FREQUENCIES[self.freq_index] + self.fei_array[self.freq_index])],
          0x09: [REG_FRFLSB, lsb(ISS_FREQUENCIES[self.freq_index] + self.fei_array[self.freq_index])],
          # Use 25 kHz BW (BitRate < 2 * RxBw)
          0x19: [REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_4],
          # Use double the bandwidth during AFC as reception
          0x1a: [REG_AFCBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_20 | RF_RXBW_EXP_3],
          0x1e: [REG_AFCFEI, RF_AFCFEI_AFCAUTOCLEAR_ON | RF_AFCFEI_AFCAUTO_ON],
          0x25: [REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01], # map DIO0 to PayloadReady IRQ
          0x29: [REG_RSSITHRESH, SENSIVITY], # Rssi Threshold
          # Davis has four preamble bytes 0xAAAAAAAA
          0x2d: [REG_PREAMBLELSB, 4], 
          # Sync detection on, FIFO filling condition 0, 2 sync words, no sync errors
          0x2e: [REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0],
          0x2f: [REG_SYNCVALUE1, RF_SYNC_BYTE1_VALUE], # Davis first sync byte 0xCB
          0x30: [REG_SYNCVALUE2, RF_SYNC_BYTE2_VALUE], # Davis second sync byte 0x89
          # Fixed packet length and we'll check our own CRC
          0x37: [REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF |
                 RF_PACKET1_CRC_OFF | RF_PACKET1_CRCAUTOCLEAR_OFF | RF_PACKET1_ADRSFILTERING_OFF],
          # Davis sends 10 bytes of payload, including CRC that we check manually
          0x38: [REG_PAYLOADLENGTH, 10],
          # RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
          0x3d: [REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF],
          # SensitivityBoost
          0x58: [REG_TESTLNA, 0x2d],
          # Improved margin
          0x6f: [REG_TESTDAGC, 0x30],
        }

        # initialize timer
        self.timer = Timer(TIMER_INTERVAL, None)

        # initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)
        self.spi.max_speed_hz = 4000000

        # verify chip is syncing?
        while self.read_register(REG_SYNCVALUE1) != RF_SYNC_BYTE1_VALUE:
            self.write_register(REG_SYNCVALUE1, RF_SYNC_BYTE1_VALUE)

        while self.read_register(REG_SYNCVALUE1) != RF_SYNC_BYTE2_VALUE:
            self.write_register(REG_SYNCVALUE1, RF_SYNC_BYTE2_VALUE)

        # write config
        for value in self.CONFIG.values():
            self.write_register(value[0], value[1])

        # wait for ModeReady
        while (self.read_register(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00:
            pass

        GPIO.remove_event_detect(IRQ_PIN)
        GPIO.add_event_detect(IRQ_PIN, GPIO.RISING, callback=self.interrupt_handler)

    def read_register(self, addr):
        return self.spi.xfer([addr & 0x7F, 0])[1]

    def write_register(self, addr, value):
        self.spi.xfer([addr | 0x80, value])

    def setmode(self, newMode):
        if newMode == self.mode:
            return
        mask = self.read_register(REG_OPMODE) & 0xE3 # 11100011
        if newMode == RF69_MODE_TX:
            self.write_register(REG_OPMODE, mask | RF_OPMODE_TRANSMITTER)
        elif newMode == RF69_MODE_RX:
            self.write_register(REG_OPMODE, mask | RF_OPMODE_RECEIVER)
        elif newMode == RF69_MODE_SYNTH:
            self.write_register(REG_OPMODE, mask | RF_OPMODE_SYNTHESIZER)
        elif newMode == RF69_MODE_STANDBY:
            self.write_register(REG_OPMODE, mask | RF_OPMODE_STANDBY)
        elif newMode == RF69_MODE_SLEEP:
            self.write_register(REG_OPMODE, mask | RF_OPMODE_SLEEP)
        else:
            return

        # we are using packet mode, so this check is not really needed
        # but waiting for mode ready is necessary when going from sleep because the FIFO may not be immediately available from previous mode
        while self.mode == RF69_MODE_SLEEP and self.read_register(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY == 0x00:
            pass

        self.mode = newMode;

    def read_rssi(self):
        return -self.read_register(REG_RSSIVALUE) / 2
    
    def read_fei(self):
        fei = (self.read_register(REG_FEIMSB) << 8) | self.read_register(REG_FEILSB)
        if fei >= 32768:
            fei = fei - 65536
        return fei

    def calibration(self):
        self.write_register(REG_OSC1, RF_OSC1_RCCAL_START)
        while self.read_register(REG_OSC1) & RF_OSC1_RCCAL_DONE == 0x00:
            pass

    def hop(self, freq_index):
        self.freq_index = freq_index
        logging.debug("hop (10" + str(self.freq_index) + ")")
        self.write_register(REG_FRFMSB,
            msb(ISS_FREQUENCIES[self.freq_index] + self.fei_array[self.freq_index]))
        self.write_register(REG_FRFMID,
            mid(ISS_FREQUENCIES[self.freq_index] + self.fei_array[self.freq_index]))
        self.write_register(REG_FRFLSB,
            lsb(ISS_FREQUENCIES[self.freq_index] + self.fei_array[self.freq_index]))
        # optimized sequence from documentation 4.2.5
        self.setmode(RF69_MODE_SYNTH)
        self.setmode(RF69_MODE_RX)

    def sleep(self):
        self.setmode(RF69_MODE_SLEEP)

    def receive_begin(self):
        #set DIO0 to "PayloadRead" in receive mode
        self.write_register(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01)
        self.setmode(RF69_MODE_RX)
        self.timer.start()

    def shutdown(self):
        self.timer.cancel()
        self.sleep()
        GPIO.cleanup()

    def set_handler(self, handler):
        self.handler = handler

    def timer_handler(self, freq_index):
        with self.timer.condition:
            self.hop_count += 1
            if self.hop_count >= MAX_HOPS:
                self.timer.callback = None
                freq_index = 0
            self.hop(freq_index)

    def interrupt_handler(self, pin):
        with self.timer.condition:
            if self.mode == RF69_MODE_RX and self.read_register(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY:
                timestamp = time.time()
                freq_index = self.freq_index
                a = self.spi.xfer2([REG_FIFO & 0x7f,0,0,0,0,0,0,0,0,0,0])[1:]
                rssi = self.read_rssi()
                fei = self.read_fei()
                message = "I " + "%3d" % (100 + freq_index)
                for idx in range(8):
                    hex = "%02X" % reverse_bits(a[idx])
                    message += " " + hex
                message += " %4s" % str(rssi)
                message += " %4s" % str(fei)
                message += " %2d" % self.hop_count
                if self.handler:
                    if self.handler(message):
                        self.fei_array[freq_index] += fei
                        self.valid_messages += 1
                        self.lost_messages += self.hop_count - 1
                        self.hop_count = 0
                        self.timer.calibrate(self.timer_handler, timestamp, freq_index)

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

# Check line for crc
def check(line):
    return crc(line.split()) == 0

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

# Parses values from line
def description(line):
    description = "unknown"
    data = line.split()

    if crc(data) != 0:
        return "invalid crc"

    elif data[0] == 'I':
        header = int(data[2], 16) >> 4
        windSpeed = int(round(int(data[3], 16) * 1.609344, 0))
        windDirections = (int(data[4], 16) << 2) | (int(data[6], 16) & 0x02)
        windDirections = 360 if windDirections > 1024 or windDirections <= 0 else int(round(windDirections * 360.0 / 1024.0))  
        windData = " w(" + str(windSpeed) + "km/h, " + str(windDirections) + ")"

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
            description = "t=" + str(temperature) + "C"

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

LOOP_TIME = 60

class DataLogger(object):
    def __init__(self):
        self.receiver = DavisReceiver()
        self.receiver.set_handler(self.process_message)

    def loop(self):
        self.receiver.calibration()
        self.receiver.receive_begin()

        # main loop
        while True:
            ts = int(time.time())
            self.sleep(ts + LOOP_TIME - ts % LOOP_TIME)

    def sleep(self, target):
        while True:
            ts = int(time.time())
            if ts < target:
                time.sleep(target - ts)
            else:
                break
            
    def shutdown(self):
        if self.receiver:
            self.receiver.shutdown()

    def process_message(self, message):
        if not check(message):
            logging.info("invalid message received: " + message + " " + description(message))
            return False

        logging.info(message + " " + description(message))
        time.sleep(0.1)
        return True

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
    except Exception as e:
        logging.critical(str(e))

    finally:
        if data_logger:
            data_logger.shutdown()
