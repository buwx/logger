#!/usr/bin/python
# -*- coding: iso-8859-15 -*-
'''
Created on 23.05.2016

@author: micha
'''

import logging
import time

# I2C address definition
BMP280_I2C_ADDRESS1 = 0x76
BMP280_I2C_ADDRESS2 = 0x77

# power mode definition
BMP280_SLEEP_MODE      = 0x00
BMP280_FORCED_MODE     = 0x01
BMP280_NORMAL_MODE     = 0x03

BMP280_SOFT_RESET_CODE = 0xB6

# register address definition
BMP280_CHIP_ID_REG          = 0xD0 # Chip ID Register
BMP280_RST_REG              = 0xE0 # Softreset Register
BMP280_STAT_REG             = 0xF3 # Status Register
BMP280_CTRL_MEAS_REG        = 0xF4 # Ctrl Measure Register
BMP280_CONFIG_REG           = 0xF5 # Configuration Register
BMP280_PRESSURE_MSB_REG     = 0xF7 # Pressure MSB Register
BMP280_PRESSURE_LSB_REG     = 0xF8 # Pressure LSB Register
BMP280_PRESSURE_XLSB_REG    = 0xF9 # Pressure XLSB Register
BMP280_TEMPERATURE_MSB_REG  = 0xFA # Temperature MSB Reg
BMP280_TEMPERATURE_LSB_REG  = 0xFB # Temperature LSB Reg
BMP280_TEMPERATURE_XLSB_REG = 0xFC # Temperature XLSB Reg

# calibration parameters
BMP280_TEMPERATURE_CALIB_DIG_T1_REG = 0x88
BMP280_TEMPERATURE_CALIB_DIG_T2_REG = 0x8A
BMP280_TEMPERATURE_CALIB_DIG_T3_REG = 0x8C
BMP280_PRESSURE_CALIB_DIG_P1_REG    = 0x8E
BMP280_PRESSURE_CALIB_DIG_P2_REG    = 0x90
BMP280_PRESSURE_CALIB_DIG_P3_REG    = 0x92
BMP280_PRESSURE_CALIB_DIG_P4_REG    = 0x94
BMP280_PRESSURE_CALIB_DIG_P5_REG    = 0x96
BMP280_PRESSURE_CALIB_DIG_P6_REG    = 0x98
BMP280_PRESSURE_CALIB_DIG_P7_REG    = 0x9A
BMP280_PRESSURE_CALIB_DIG_P8_REG    = 0x9C
BMP280_PRESSURE_CALIB_DIG_P9_REG    = 0x9E

# oversampling definition
BMP280_OVERSAMP_SKIPPED = 0x00
BMP280_OVERSAMP_1X      = 0x01
BMP280_OVERSAMP_2X      = 0x02
BMP280_OVERSAMP_4X      = 0x03
BMP280_OVERSAMP_8X      = 0x04
BMP280_OVERSAMP_16X     = 0x05

# working mode definition
BMP280_ULTRA_LOW_POWER_MODE       = 0x00
BMP280_LOW_POWER_MODE             = 0x01
BMP280_STANDARD_RESOLUTION_MODE   = 0x02
BMP280_HIGH_RESOLUTION_MODE       = 0x03
BMP280_ULTRA_HIGH_RESOLUTION_MODE = 0x04

BMP280_ULTRALOWPOWER_OVERSAMP_PRESSURE          = BMP280_OVERSAMP_1X
BMP280_ULTRALOWPOWER_OVERSAMP_TEMPERATURE       = BMP280_OVERSAMP_1X
BMP280_LOWPOWER_OVERSAMP_PRESSURE               = BMP280_OVERSAMP_2X
BMP280_LOWPOWER_OVERSAMP_TEMPERATURE            = BMP280_OVERSAMP_1X
BMP280_STANDARDRESOLUTION_OVERSAMP_PRESSURE     = BMP280_OVERSAMP_4X
BMP280_STANDARDRESOLUTION_OVERSAMP_TEMPERATURE  = BMP280_OVERSAMP_1X
BMP280_HIGHRESOLUTION_OVERSAMP_PRESSURE         = BMP280_OVERSAMP_8X
BMP280_HIGHRESOLUTION_OVERSAMP_TEMPERATURE      = BMP280_OVERSAMP_1X
BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE    = BMP280_OVERSAMP_16X
BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE = BMP280_OVERSAMP_2X

# filter definition
BMP280_FILTER_COEFF_OFF = 0x00
BMP280_FILTER_COEFF_2   = 0x01
BMP280_FILTER_COEFF_4   = 0x02
BMP280_FILTER_COEFF_8   = 0x03
BMP280_FILTER_COEFF_16  = 0x04

# standby definition
BMP280_STANDBY_000  = 0   # 0.5 ms
BMP280_STANDBY_001  = 1   # 62.5 ms
BMP280_STANDBY_010  = 2   # 125 ms
BMP280_STANDBY_011  = 3   # 250 ms
BMP280_STANDBY_100  = 4   # 500 ms
BMP280_STANDBY_101  = 5   # 1000 ms
BMP280_STANDBY_110  = 6   # 2000 ms
BMP280_STANDBY_111  = 7   # 4000 ms

# sensor ID
BMP280_SENSOR_ID = 0x58

class BMP280(object):
    def __init__(self, mode=BMP280_NORMAL_MODE, address=BMP280_I2C_ADDRESS1, i2c=None, **kwargs):
        self._logger = logging.getLogger('BMP280')
        # check that mode is valid.
        if mode not in [BMP280_SLEEP_MODE, BMP280_FORCED_MODE, BMP280_NORMAL_MODE]:
            raise ValueError(
                'Unexpected mode value {0}. Set mode to one of BMP280_SLEEP_MODE, BMP280_FORCED_MODE or BMP280_NORMAL_MODE'.format(mode))
        self._mode = mode
        # create I2C device.
        if i2c is None:
            import Adafruit_GPIO.I2C as I2C
            i2c = I2C
        self._device = i2c.get_i2c_device(address, **kwargs)

        # check device id
        if self._device.readS8(BMP280_CHIP_ID_REG) != BMP280_SENSOR_ID:
            raise Exception('Wrong pressure sensor')

        # soft reset sensor
        self._device.write8(BMP280_RST_REG, BMP280_SOFT_RESET_CODE)
        time.sleep(0.2) # wait

        # set control measurement value 
        ctrl_meas = (BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE << 5) | (BMP280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE << 2) | self._mode
        self._device.write8(BMP280_CTRL_MEAS_REG, ctrl_meas)
        time.sleep(0.2) # wait

        # set configuration value
        config = (BMP280_STANDBY_100 << 5) | (BMP280_FILTER_COEFF_16 << 2)
        self._device.write8(BMP280_CONFIG_REG, config)  #
        time.sleep(0.2) # wait

        # load calibration values.
        self._load_calibration()

    def _load_calibration(self):
        self._dig_T1 = self._device.readU16LE(BMP280_TEMPERATURE_CALIB_DIG_T1_REG) # read correction settings
        self._dig_T2 = self._device.readS16LE(BMP280_TEMPERATURE_CALIB_DIG_T2_REG)
        self._dig_T3 = self._device.readS16LE(BMP280_TEMPERATURE_CALIB_DIG_T3_REG)
        self._dig_P1 = self._device.readU16LE(BMP280_PRESSURE_CALIB_DIG_P1_REG)
        self._dig_P2 = self._device.readS16LE(BMP280_PRESSURE_CALIB_DIG_P2_REG)
        self._dig_P3 = self._device.readS16LE(BMP280_PRESSURE_CALIB_DIG_P3_REG)
        self._dig_P4 = self._device.readS16LE(BMP280_PRESSURE_CALIB_DIG_P4_REG)
        self._dig_P5 = self._device.readS16LE(BMP280_PRESSURE_CALIB_DIG_P5_REG)
        self._dig_P6 = self._device.readS16LE(BMP280_PRESSURE_CALIB_DIG_P6_REG)
        self._dig_P7 = self._device.readS16LE(BMP280_PRESSURE_CALIB_DIG_P7_REG)
        self._dig_P8 = self._device.readS16LE(BMP280_PRESSURE_CALIB_DIG_P8_REG)
        self._dig_P9 = self._device.readS16LE(BMP280_PRESSURE_CALIB_DIG_P9_REG)
        self._t_fine = 117300.0

    def read_raw_temp(self):
        """Reads the raw (uncompensated) temperature from the sensor."""
        raw_temp_msb = self._device.readU8(BMP280_TEMPERATURE_MSB_REG)   # read raw temperature msb
        raw_temp_lsb = self._device.readU8(BMP280_TEMPERATURE_LSB_REG)   # read raw temperature lsb
        raw_temp_xlsb = self._device.readU8(BMP280_TEMPERATURE_XLSB_REG) # read raw temperature xlsb
        raw_temp = (raw_temp_msb << 12) | (raw_temp_lsb << 4) | (raw_temp_xlsb >> 4)
        self._logger.debug('Raw temp 0x{0:X} ({1})'.format(raw_temp & 0xFFFF, raw_temp))
        return raw_temp

    def read_raw_pressure(self):
        """Reads the raw (uncompensated) pressure level from the sensor."""
        raw_press_msb = self._device.readU8(BMP280_PRESSURE_MSB_REG)   # read raw temperature msb
        raw_press_lsb = self._device.readU8(BMP280_PRESSURE_LSB_REG)   # read raw temperature lsb
        raw_press_xlsb = self._device.readU8(BMP280_PRESSURE_XLSB_REG) # read raw temperature xlsb
        raw_press = (raw_press_msb << 12) | (raw_press_lsb << 4) | (raw_press_xlsb >> 4)
        self._logger.debug('Raw pressure 0x{0:04X} ({1})'.format(raw_press & 0xFFFF, raw_press))
        return raw_press

    def read_temperature(self):
        """Gets the compensated temperature in degrees celsius."""
        raw_temp = self.read_raw_temp()
        # calculate x1
        v_x1 = (raw_temp / 16384.0 - self._dig_T1 / 1024.0) * self._dig_T2
        # calculate x2
        v_x2 = (raw_temp / 131072.0 - self._dig_T1 / 8192.0) * (raw_temp / 131072.0 - self._dig_T1 / 8192.0) * self._dig_T3
        # calculate t_fine
        self._t_fine = v_x1 + v_x2
        temperature = self._t_fine / 5120.0
        self._logger.debug('Calibrated temperature {0} C'.format(temperature))
        return temperature

    def read_pressure(self):
        """Gets the compensated pressure in Pascals."""
        raw_press = self.read_raw_pressure()
        v_x1 = self._t_fine / 2.0 - 64000.0
        v_x2 = v_x1 * v_x1 * self._dig_P6 / 32768.0
        v_x2 = v_x2 + v_x1 * self._dig_P5 * 2.0
        v_x2 = v_x2 / 4.0 + self._dig_P4 * 65536.0
        v_x1 = (self._dig_P3 * v_x1 * v_x1 / 524288.0 + self._dig_P2 * v_x1) / 524288.0
        v_x1 = (1.0 + v_x1 / 32768.0) * self._dig_P1
        pressure = 1048576.0 - raw_press
        # avoid exception caused by division by zero
        if v_x1 != 0.0:
            self._logger.warning('invalid pressure')
            pressure = (pressure - v_x2 / 4096.0) * 6250.0 / v_x1;
        else
            return 0.0; # invalid pressure

        v_x1 = self._dig_P9 * pressure * pressure / 2147483648.0
        v_x2 = pressure * self._dig_P8 / 32768.0
        pressure = pressure + (v_x1 + v_x2 + self._dig_P7) / 16.0
        self._logger.debug('Pressure {0} Pa'.format(pressure))
        return pressure

    def read_altitude(self, sealevel_pa = 101325.0):
        """Calculates the altitude in meters."""
        # Calculation taken straight from section 3.6 of the datasheet.
        pressure = self.read_pressure()
        altitude = 44330.0 * (1.0 - pow(pressure / sealevel_pa, (1.0/5.255)))
        self._logger.debug('Altitude {0} m'.format(altitude))
        return altitude

    def read_sealevel_pressure(self, altitude_m=0.0):
        """Calculates the pressure at sealevel when given a known altitude in
        meters. Returns a value in Pascals."""
        pressure = self.read_pressure()
        p0 = pressure / pow(1.0 - altitude_m/44330.0, 5.255)
        self._logger.debug('Sealevel pressure {0} Pa'.format(p0))
        return p0
