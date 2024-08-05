import numpy as np
import matplotlib.pyplot as plt
import os

make_plots=False

c=3e8
n=1.8
sampling_rate=118e6*4
int_factor=4
int_rate=sampling_rate*int_factor
num_antennas=4

cable_delays=np.array([716.2603798064285,711.7615958304959,706.495904921158,702.195731800799])
ant_depths=np.array([-96.215,-95.174,-94.183,-93.155])

def get_delay(ant_top=0,ant_num=0,angle=0):
    return (ant_depths[ant_top]-ant_depths[ant_num])*np.cos((90-angle)*np.pi/180)*n/c+(cable_delays[ant_num]-cable_delays[ant_top])/1e9



angs=np.linspace(-80,80,160)
delays=np.zeros((4,len(angs)))
lookback=np.zeros((4,len(angs)))

delays[0]=get_delay(3,0,angs)
delays[1]=get_delay(3,1,angs)
delays[2]=get_delay(3,2,angs)
delays[3]=get_delay(3,3,angs)

for i in range(4):
    lookback[i]=-(delays[i]-np.max(delays.T,axis=1))

if make_plots:
    if not os.path.exists('plots'): os.mkdir('plots')
    plt.figure()
    plt.plot(angs,delays[0],label='ch03')
    plt.plot(angs,delays[1],label='ch13')
    plt.plot(angs,delays[2],label='ch23')
    plt.plot(angs,delays[3],label='ch33')

    plt.xlabel('angles (deg)')
    plt.ylabel('delays (s)')
    plt.legend()
    plt.savefig('plots/arrival_times.png')
    plt.close()

    plt.figure()
    plt.plot(angs,lookback[0],label='ch03')
    plt.plot(angs,lookback[1],label='ch13')
    plt.plot(angs,lookback[2],label='ch23')
    plt.plot(angs,lookback[3],label='ch33')

    plt.xlabel('angles (deg)')
    plt.ylabel('lookback (s)')
    plt.legend()
    plt.savefig('plots/lookback_times.png')
    plt.close()

    plt.figure()
    plt.plot(angs,lookback[0]*int_rate,label='ch0')
    plt.plot(angs,lookback[1]*int_rate,label='ch1')
    plt.plot(angs,lookback[2]*int_rate,label='ch2')
    plt.plot(angs,lookback[3]*int_rate,label='ch3')

    plt.xlabel('angles (deg)')
    plt.ylabel('lookback (int samples)')
    plt.legend()
    plt.savefig('plots/lookback_interpolated_samples.png')
    plt.close()


num_beams=8
beam_locs=np.linspace(-60,60,num_beams)
print('beam locs',beam_locs)
beam_lookback=np.zeros((4,num_beams))
beam_lookback[0]=np.round(np.interp(beam_locs,angs,lookback[0]*int_rate))+15
beam_lookback[1]=np.round(np.interp(beam_locs,angs,lookback[1]*int_rate))+15
beam_lookback[2]=np.round(np.interp(beam_locs,angs,lookback[2]*int_rate))+15
beam_lookback[3]=np.round(np.interp(beam_locs,angs,lookback[3]*int_rate))+15

if make_plots:
    plt.figure()
    plt.scatter(beam_locs,beam_lookback[0],label='ch0')
    plt.scatter(beam_locs,beam_lookback[1],label='ch1')
    plt.scatter(beam_locs,beam_lookback[2],label='ch2')
    plt.scatter(beam_locs,beam_lookback[3],label='ch3')

    plt.xlabel('angles (deg)')
    plt.ylabel('lookback (int samples)')
    plt.legend()
    plt.savefig('plots/beam_lookback_samples.png')
    plt.close()



print('print out for quartus')
print('(',end='')
for i in range(num_beams):
    print('(%i,%i,%i,%i)'%(beam_lookback[3][i],beam_lookback[2][i],beam_lookback[1][i],beam_lookback[0][i]),end='')
    if i==num_beams-1:
        break
    print(',',end='')
print(');',end='')
print()



