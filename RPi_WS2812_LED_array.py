import time 
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
import math
import board
import neopixel
pixels=neopixel.NeoPixel(board.D18, 256)
#phi=azimuthal angle
#theta=polar angle
#phase centre:(7.5,7.5) i.e. centre of board
#(0,0) is located on LED 16 (in the corner)
#red:phase behind of phase centre
#blue:phase ahead of phase centre
#therefore beam points in direction of red

l=2 #wavelength in cm
s=1 #element separation in cm
brightness=10 #max 255
variance=37.5 #of the array tapering from centre (variance=standard deviation squared)
t=0 #sleep time between updates

def phasestep(theta,phi):
    alpha=np.arctan2(1/np.tan(np.deg2rad(theta)),np.cos(np.deg2rad(phi)))
    beta=np.arctan2(1/np.tan(np.deg2rad(theta)),np.sin(np.deg2rad(phi)))
    dx=(s*np.cos(alpha)/l)
    dy=(s*np.cos(beta)/l)

    return [dx,dy] #phase step between adjacent elements in x and y direction


def array2d(theta,phi):#
    dx=phasestep(theta,phi)[0]
    dy=phasestep(theta,phi)[1]
    data=np.zeros((16,16))
    for i in range(16):
        for j in range(16):
            data[i,j]=-(i-7.5)*dx-(j-7.5)*dy #since phase centre at (7.5,7.5)

    return data #16x16 2d array of the phase difference from phase centre of every element

def light(theta,phi):
    c=array2d(theta,phi)
    for i in range(256):
        x=i%16
        y=int((i-x)/16)
        if y%2==1:
            x=15-x #x direction must be flipped every other row as the order of LEDs snakes along
        r_squared=(x-7.5)**2+(y-7.5)**2
        amplitude=math.exp(-(r_squared)/(2*variance))*brightness #Gaussian array tapering (112.5)
        t=(c[x][y]/array2d(70,45)[-1][-1]) #phase/max phase (-max phase<phase<max phase) !!!!
        u=0.5*t+0.5 #make this a number between 0 and 1
        red=round(u*amplitude) #make large phase difference more red
        blue=round((1-u)*amplitude) #make small phase difference more blue
        pixels[i]=(red,0,blue) #(red,green,blue)

def animate(x): #update the LED matrix live for many pairs of angles
    for i in range(x.shape[0]):
        light(x[i][0],x[i][1])
        time.sleep(t)

steps=4
theta_initial=70
theta_final=70
theta_step=(theta_final-theta_initial)/(steps-1)
phi_initial=45
phi_final=135
phi_step=(phi_final-phi_initial)/(steps-1)

x=np.zeros((steps,2)) #pairs of angles
for i in range(steps):
    x[i][0]=theta_initial+i*theta_step
for j in range(steps):
    x[j][-1]=phi_initial+j*phi_step


#light(70,45)
print(x)
animate(x)
