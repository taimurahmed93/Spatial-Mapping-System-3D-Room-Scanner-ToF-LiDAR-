# Taimur Ahmed 400514463
#2DX3
import serial
import math
s = serial.Serial('COM7',115200, timeout=10) 

s.open
s.reset_output_buffer()
s.reset_input_buffer()
f = open("point_array.xyz", "w") 
st = 0
x = 0 
increment = 250 
num_inc = int(input("How many scans do you want to take:"))
num=0
while(num<num_inc):
    raw = s.readline()
    measurement = raw.decode("utf-8") 
    measurement = measurement[0:-2] 

    if (measurement.isdigit() == True):
        angle = (st/512)*2*math.pi 
        r = int(measurement)
        y = r*math.cos(angle) 
        z = r*math.sin(angle) 
        print(y)
        print(z)
        f.write('{} {} {}\n'.format(x,y,z)) 
        st = st+ 32
    if (st == 512): 
        st = 0
        x = x + increment
        num=num+1
    print(measurement)
f.close() 

