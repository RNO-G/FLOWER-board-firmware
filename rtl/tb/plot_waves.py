import numpy as np
import matplotlib.pyplot as plt

input_data=np.loadtxt("data/plot_input_waveforms.txt")



f=open("data/output_upsampled.txt")
up_data=np.zeros((4,1024*4))
for i in range(256):
    line=f.readline()
    vals=(line.split(" "))[0:64]

    for j in range(64):
        val=(int(vals[j],2)-128)#output_data[i][j]-128
        ch=int(np.trunc(j/16))
        sam=16*i+(15-j % 16)
        #print(ch,sam)
        up_data[ch][sam]=val

        #if j<16:
        #    up_data[0][i*16+(16-j)]=val
        #elif j<32:
        #    up_data[1][i*16+(16-j)-16]=val
        #elif j<48:
        #    up_data[2][i*16+(16-j)-32]=val
        #elif j<64:
        #    up_data[3][i*16+(16-j)-48]=val


f=open("data/output_beamformed.txt")
beam_data=np.zeros((12,1024*4))
for i in range(256):
    line=f.readline()
    vals=(line.split(" "))[0:12*16]
    #vals=vals[::-1]

    for j in range(12*16):
        val=(int(vals[j],2)-128)#output_data[i][j]-128
        bm=int(np.trunc(j/16))
        sam=16*i+(15-j % 16)
        #print(bm,sam)
        beam_data[bm][sam]=val
    #input()
f=open("data/output_power.txt")
power_data=np.zeros((12,int(1024/2)))
for i in range(256):
    line=f.readline()
    vals=(line.split(" "))[0:2*12]
    #vals=vals[::-1]

    for j in range(12*2):
        val=(int(vals[j],2))#output_data[i][j]-128
        bm=int(np.trunc(j/2))
        sam=2*i+(1-j % 2)
        #print(bm,sam)
        power_data[bm][sam]=val

trigs=np.loadtxt("data/output_trigger.txt")

t_base=np.arange(0,1024,1)/.472
t_up=np.arange(0,1024,.25)/.472#-29.25
t_beamformed=np.arange(0,1024,.25)/.472#-29.25
t_power=np.arange(0,1024,2)/.472#-29.25
t_trig=np.arange(0,1024,4)/.472

ts_base=np.arange(0,1024,1)
ts_up=np.arange(0,1024,.25)-25.25

plt.figure()
plt.plot(ts_base,input_data[3],label="ch %i base"%3)
plt.plot(ts_up,up_data[3],label="ch %i upsampled"%3)

plt.legend(loc="right")
plt.show()


plt.figure()
for i in range(4):
    plt.plot(t_base,input_data[i],label="ch %i base"%i)
    plt.plot(t_up,up_data[i],label="ch %i upsampled"%i)

for i in range(12):
    plt.plot(t_beamformed,beam_data[i],label="beam %i wave"%i)

for i in range(12):
    plt.plot(t_power,power_data[i],label="bm %i power"%i)

plt.plot(t_trig,trigs*500,label="triggers")
plt.xlabel("time (ns)")
plt.legend(loc="right")
plt.show()