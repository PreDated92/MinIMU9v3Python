#Driver for the LSM303D accelerometer and magnetometercompass

#First follow the procedure to enable I2C on R-Pi.
#1. Add the lines ic2-bcm2708 and i2c-dev to the file etcmodules
#2. Comment out the line blacklist ic2-bcm2708 (with a #) in the file etcmodprobe.draspi-blacklist.conf
#3. Install I2C utility (including smbus) with the command apt-get install python-smbus i2c-tools
#4. Connect the I2C device and detect it using the command i2cdetect -y 1.  It should show up as 1D or 1E (here the variable LSM is set to 1D).

#Driver by Fayetteville Free Library Robotics Group

import time, math
from smbus import SMBus
busNum = 1
b = SMBus(busNum)

def twos_comp_combine(msb, lsb):
    twos_comp = 256*msb + lsb
    if twos_comp >= 32768:
        return twos_comp - 65536
    else:
        return twos_comp

## LSM303D Registers
LSM = 0x1d #I2C Address of the LSM303D
LSM_WHOAMI_ID = 0b1001001 #Device self-id
LSM_WHOAMI_ADDRESS = 0x0F

#Control register addresses -- from LSM303D datasheet
CTRL_0 = 0x1F #General settings
CTRL_1 = 0x20 #Turns on accelerometer and configures data rate
CTRL_2 = 0x21 #Self test accelerometer, anti-aliasing accel filter
CTRL_3 = 0x22 #Interrupts
CTRL_4 = 0x23 #Interrupts
CTRL_5 = 0x24 #Turns on temperature sensor
CTRL_6 = 0x25 #Magnetic resolution selection, data rate config
CTRL_7 = 0x26 #Turns on magnetometer and adjusts mode

#Registers holding twos-complemented MSB and LSB of magnetometer readings -- from LSM303D datasheet
MAG_X_LSB = 0x08 # x
MAG_X_MSB = 0x09
MAG_Y_LSB = 0x0A # y
MAG_Y_MSB = 0x0B
MAG_Z_LSB = 0x0C # z
MAG_Z_MSB = 0x0D

#Registers holding twos-complemented MSB and LSB of magnetometer readings -- from LSM303D datasheet
ACC_X_LSB = 0x28 # x
ACC_X_MSB = 0x29
ACC_Y_LSB = 0x2A # y
ACC_Y_MSB = 0x2B
ACC_Z_LSB = 0x2C # z
ACC_Z_MSB = 0x2D

#Registers holding 12-bit right justified, twos-complemented temperature data -- from LSM303D datasheet
TEMP_MSB = 0x05
TEMP_LSB = 0x06

## L3GD20H registers

LGD = 0x6b #Device I2C slave address
LGD_WHOAMI_ADDRESS = 0x0F
LGD_WHOAMI_ID = 0b11010111 #Device self-id

LGD_CTRL_1 = 0x20 #turns on gyro
LGD_CTRL_2 = 0x21 #can set a high-pass filter for gyro
LGD_CTRL_3 = 0x22
LGD_CTRL_4 = 0x23
LGD_CTRL_5 = 0x24
LGD_CTRL_6 = 0x25

LGD_TEMP = 0x26

#Registers holding gyroscope readings
LGD_GYRO_X_LSB = 0x28
LGD_GYRO_X_MSB = 0x29
LGD_GYRO_Y_LSB = 0x2A
LGD_GYRO_Y_MSB = 0x2B
LGD_GYRO_Z_LSB = 0x2C
LGD_GYRO_Z_MSB = 0x2D

if b.read_byte_data(LSM, LSM_WHOAMI_ADDRESS) == LSM_WHOAMI_ID:
    print 'LSM303D detected successfully.'
else:
    print 'No LSM303D detected on bus '+str(busNum)+'.'
if b.read_byte_data(LGD, LGD_WHOAMI_ADDRESS) == LGD_WHOAMI_ID:
    print 'L3GD20H detected successfully.'
else:
    print 'No L3GD20H detected on bus on I2C bus '+str(busNum)+'.'

b.write_byte_data(LSM, CTRL_1, 0b1010111) # enable accelerometer, 50 hz sampling
b.write_byte_data(LSM, CTRL_2, 0x00) #set +- 2g full scale
b.write_byte_data(LSM, CTRL_5, 0b01100100) #high resolution mode, thermometer off, 6.25hz ODR
b.write_byte_data(LSM, CTRL_6, 0b00100000) # set +- 4 gauss full scale
b.write_byte_data(LSM, CTRL_7, 0x00) #get magnetometer out of low power mode

b.write_byte_data(LGD, LGD_CTRL_1, 0x0F) #turn on gyro and set to normal mode

DT = 0.5
PI = 3.14159265358979323846
RAD_TO_DEG = 57.29578
AA = 0.98
gyrox_angle = 0.0;
gyroy_angle = 0.0;
gyroz_angle = 0.0;

while True:
    time.sleep(0.5)
    magx = twos_comp_combine(b.read_byte_data(LSM, MAG_X_MSB), b.read_byte_data(LSM, MAG_X_LSB))
    magy = twos_comp_combine(b.read_byte_data(LSM, MAG_Y_MSB), b.read_byte_data(LSM, MAG_Y_LSB))
    magz = twos_comp_combine(b.read_byte_data(LSM, MAG_Z_MSB), b.read_byte_data(LSM, MAG_Z_LSB))

    #print "Magnetic field (x, y, z):", magx, magy, magz

    accx = twos_comp_combine(b.read_byte_data(LSM, ACC_X_MSB), b.read_byte_data(LSM, ACC_X_LSB))
    accy = twos_comp_combine(b.read_byte_data(LSM, ACC_Y_MSB), b.read_byte_data(LSM, ACC_Y_LSB))
    accz = twos_comp_combine(b.read_byte_data(LSM, ACC_Z_MSB), b.read_byte_data(LSM, ACC_Z_LSB))

    #print "Acceleration (x, y, z):", accx, accy, accz

    gyrox = twos_comp_combine(b.read_byte_data(LGD, LGD_GYRO_X_MSB), b.read_byte_data(LGD, LGD_GYRO_X_LSB))
    gyroy = twos_comp_combine(b.read_byte_data(LGD, LGD_GYRO_Y_MSB), b.read_byte_data(LGD, LGD_GYRO_Y_LSB))
    gyroz = twos_comp_combine(b.read_byte_data(LGD, LGD_GYRO_Z_MSB), b.read_byte_data(LGD, LGD_GYRO_Z_LSB))
    
    #print "Gyroscope (x, y, z):", gyrox, gyroy, gyroz
    rate_gyrox = gyrox * 0.00875
    rate_gyroy = gyroy * 0.00875
    rate_gyroz = gyroz * 0.00875
    
    gyrox_angle+=rate_gyrox*DT;
    gyroy_angle+=rate_gyroy*DT;
    gyroz_angle+=rate_gyroz*DT;
    
    accx_angle = (atan2(accy,accz)+PI)*RAD_TO_DEG;
    accy_angle = (atan2(accz,accx)+PI)*RAD_TO_DEG;
    
    CFangx = AA*(CFangx+gyrox_angle) +(1 - AA) * accx_angle;
    CFangy = AA*(CFangy+gyroy_angle) +(1 - AA) * accy_angle;

    print "Angle = ", CFangx, CFangy
    