import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import math

l=2 #wavelength in cm
s=1 #element separation in cm
def phasestep(theta,phi):
    alpha=np.arctan2(1/np.tan(np.deg2rad(theta)),np.cos(np.deg2rad(phi)))
    beta=np.arctan2(1/np.tan(np.deg2rad(theta)),np.sin(np.deg2rad(phi)))
    dx=(s*np.cos(alpha)/l)
    dy=(s*np.cos(beta)/l)
    
    
    return [dx,dy]

def array2d(theta,phi):
    dx=phasestep(theta,phi)[0]
    dy=phasestep(theta,phi)[1]
    data=np.zeros((16,16))
    for i in range(16):
        for j in range(16):
            data[i,j]=-(i-7.5)*dx-(j-7.5)*dy #phase centre at (7.5,7.5)
    return data

def plotarray1(theta,phi):
    c=array2d(theta,phi)
    plt.figure(figsize=(6,6))
    plt.plot(7.5,7.5,marker="o",markersize=8,color=cm.rainbow(0.05),label='phase=6 wavelengths')
    plt.plot(7.5,7.5,marker="o",markersize=8,color=cm.rainbow(0.95),label='phase=-6 wavelengths')
    for i in range(16):
        for j in range(16):
            t=c[i][j]/array2d(70,45)[-1][-1]
            u=0.5*t+0.5
            colour=cm.rainbow(u) #why does phi=135 fuck it up? t+0.5
            plt.plot(i,j,marker="s",markersize=17,color=colour)#,alpha=math.exp(-((i-7.5)**2+(j-7.5)**2)/112.5))
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('$\\theta}$=%i$^\circ$ $\phi$=%i$^\circ$' %(theta,phi))
    plt.plot(7.5,7.5,marker="o",markersize=8,color="black",label='phase centre')
    plt.legend(loc=(1,0.85))
    plt.show()

phi=np.linspace(0,90,4)
for i in range(len(phi)):
    plotarray1(70,phi[i])