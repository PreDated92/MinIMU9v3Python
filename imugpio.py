#Driver for the LSM303D accelerometer and L3GD20H magnetometer and compass

#First follow the procedure to enable I2C on R-Pi.
#1. Add the lines ic2-bcm2708 and i2c-dev to the file etcmodules
#2. Comment out the line blacklist ic2-bcm2708 (with a #) in the file etcmodprobe.draspi-blacklist.conf
#3. Install I2C utility (including smbus) with the command apt-get install python-smbus i2c-tools
#4. Connect the I2C device and detect it using the command i2cdetect -y 1.  It should show up as 1D or 1E (here the variable LSM is set to 1D).

import time, math
import wiringpi2 as wiringpi
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

b.write_byte_data(LSM, CTRL_1, 0b1100111) # enable accelerometer, 100 hz sampling
b.write_byte_data(LSM, CTRL_2, 0b0000000) #set +- 2g full scale page 36 datasheet
b.write_byte_data(LSM, CTRL_5, 0b01100100) #high resolution mode, thermometer off, 6.25hz ODR
b.write_byte_data(LSM, CTRL_6, 0b00100000) # set +- 4 gauss full scale
b.write_byte_data(LSM, CTRL_7, 0x00) #get magnetometer out of low power mode

b.write_byte_data(LGD, LGD_CTRL_1, 0x0F) #turn on gyro and set to normal mode
b.write_byte_data(LGD, LGD_CTRL_4, 0b00110000) #set 2000 dps full scale

wiringpi.wiringPiSetup()
wiringpi.pinMode(0, 1) # sets WP pin 0 to output
wiringpi.pinMode(1, 1) # sets WP pin 1 to output
wiringpi.pinMode(4, 1) # sets WP pin 4 to output ;;2 for PWM mode

DT = 0.01
PI = 3.14159265358979323846
RAD_TO_DEG = 57.29578
AA = 0.98
gyrox_angle = 0.0
gyroy_angle = 0.0
gyroz_angle = 0.0
CFangx = 0.0
CFangy = 0.0

while True:
    now = time.clock() #use process time instead of wall time, change to wall time later
    #magx = twos_comp_combine(b.read_byte_data(LSM, MAG_X_MSB), b.read_byte_data(LSM, MAG_X_LSB))
    #magy = twos_comp_combine(b.read_byte_data(LSM, MAG_Y_MSB), b.read_byte_data(LSM, MAG_Y_LSB))
    #magz = twos_comp_combine(b.read_byte_data(LSM, MAG_Z_MSB), b.read_byte_data(LSM, MAG_Z_LSB))
    #print "Magnetic field (x, y, z):", magx, magy, magz
    accx = twos_comp_combine(b.read_byte_data(LSM, ACC_X_MSB), b.read_byte_data(LSM, ACC_X_LSB))
    accy = twos_comp_combine(b.read_byte_data(LSM, ACC_Y_MSB), b.read_byte_data(LSM, ACC_Y_LSB))
    accz = twos_comp_combine(b.read_byte_data(LSM, ACC_Z_MSB), b.read_byte_data(LSM, ACC_Z_LSB))
    accx = accx * 0.061 * 0.001
    accy = accy * 0.061 * 0.001
    accz = accz * 0.061 * 0.001 - 0.1

    print "Acceleration (x, y, z):", accx, accy, accz
    gyrox = twos_comp_combine(b.read_byte_data(LGD, LGD_GYRO_X_MSB), b.read_byte_data(LGD, LGD_GYRO_X_LSB))
    gyroy = twos_comp_combine(b.read_byte_data(LGD, LGD_GYRO_Y_MSB), b.read_byte_data(LGD, LGD_GYRO_Y_LSB))
    gyroz = twos_comp_combine(b.read_byte_data(LGD, LGD_GYRO_Z_MSB), b.read_byte_data(LGD, LGD_GYRO_Z_LSB))
    #print "Gyroscope (x, y, z):", gyrox, gyroy, gyroz
    rate_gyrox = gyrox * 0.07
    rate_gyroy = gyroy * 0.07
    rate_gyroz = gyroz * 0.07
    gyrox_angle+=rate_gyrox*DT
    gyroy_angle+=rate_gyroy*DT
    gyroz_angle+=rate_gyroz*DT
    
    #accx_angle = (math.atan2(accy,math.sqrt(accx*accx+accz*accz))+PI)*RAD_TO_DEG
    accx_angle = (math.atan2(accy,accz))*RAD_TO_DEG
    #accx_angle = (math.atan2(accy,accz)+PI)*RAD_TO_DEG
    #accy_angle = (math.atan2(accx,math.sqrt(accy*accy+accz*accz))+PI)*RAD_TO_DEG
    accy_angle = (math.atan2(-accx,accz))*RAD_TO_DEG
    
    CFangx = AA*(CFangx+rate_gyrox*DT) +(1 - AA) * accx_angle
    #CFangy = AA*(CFangy+rate_gyroy*DT) +(1 - AA) * accy_angle

    print "Angle = ", CFangx, CFangy # accx_angle,accy_angle #
    
    if (CFangx < 0) :
        wiringpi.digitalWrite(0, 1)
        wiringpi.digitalWrite(1, 0)
        wiringpi.digitalWrite(4, 1)
    else :
        wiringpi.digitalWrite(0, 1)
        wiringpi.digitalWrite(1, 1)
        wiringpi.digitalWrite(4, 0)
        
    while (time.clock() <= now + DT):
        pass
