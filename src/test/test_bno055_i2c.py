#!/usr/bin/env python

import time

from src.test.BNO055_I2C import *

if __name__ == '__main__':

    bno_28 = BNO055_I2C(sensor_id=-1, address=0x28)
    if bno_28.begin() is not True:
        print "Error initializing device 28"

    bno_29 = BNO055_I2C(sensor_id=-1, address=0x29)
    if bno_29.begin() is not True:
        print "Error initializing device 29"

    time.sleep(0.1)

    bno_28.setExternalCrystalUse(True)
    bno_29.setExternalCrystalUse(True)

    while True:

        print '28: '
        #print bno_28.getVector(BNO055_I2C.VECTOR_EULER)
        print bno_28.read_euler()

        #print '29: '
        #print bno_29.getVector(BNO055_I2C.VECTOR_EULER)

        time.sleep(0.03)