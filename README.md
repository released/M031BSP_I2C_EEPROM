# M031BSP_I2C_EEPROM
 M031BSP_I2C_EEPROM


update @ 2021/05/25

1. add I2C0 PIN define (PC0 : SDA , PC1 : SCL)

2. below is eeprom test screen capture , 

![image](https://github.com/released/M031BSP_I2C_EEPROM/blob/main/eeprom_test.jpg)


update @ 2020/11/30

1. USE I2C1 (PB0 : SDA , PB1 : SCL) to initial EEPROM

2. USE terminal to printf message 

digit 1 : to dump EEPROM data

digit 2 : fix vaule (0x01) , to incr address

digit 3 : incr vaule , to fix address (0x10)

digit 4 : write array data into EEPROM

5. below is terminal screen capture

press digit 4 , to write data

![image](https://github.com/released/M031BSP_I2C_EEPROM/blob/main/WR.jpg)


press digit 1 , to dump EEPROM data

![image](https://github.com/released/M031BSP_I2C_EEPROM/blob/main/RD.jpg)
