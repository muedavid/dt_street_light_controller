import smbus

MyBus=smbus.SMBus(1)

red = 255.0
green = 255.0
blue = 200.0
for j in range (12):
        numer=250.0-j*25.0
        for i in range (16):
                MyBus.write_byte_data(0x40,3*i,int(green/255.0*numer))
                MyBus.write_byte_data(0x40,3*i+1,int(red/255.0*numer))
                MyBus.write_byte_data(0x40,3*i+2,int(blue/255.0*numer))

        try:
                input("next round: press enter")
        except:
                pass

