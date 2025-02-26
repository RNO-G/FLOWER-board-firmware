import numpy as np
import matplotlib.pyplot as plt
import os
import json
from NuRadioReco.detector.detector import Detector
from datetime import datetime as dt
make_plots=True
print_for_quartus=True

c=3e8
n=1.75
sampling_rate=118e6*4
int_factor=4
int_rate=sampling_rate*int_factor
num_antennas=4
file='RNO_season_2024.json'
det=Detector(file,source="json")

det.update(dt.now())

#print(det)
#print(det.get_channel(11,0))


stations=[11,12,13,14,21,22,23,24]
channels=[0,1,2,3]

all_delays=np.zeros((len(stations),len(channels)))
all_depths=np.zeros((len(stations),len(channels)))
num_beams=12

for i in range(len(stations)):
    for j in range(len(channels)):

        all_delays[i,j]=det.get_channel(stations[i],channels[j])['cab_time_delay']#data['channels']['%i'%pa_channels[i][j]]['cab_time_delay']
        all_depths[i,j]=det.get_channel(stations[i],channels[j])['ant_position_z']#data['channels']['%i'%pa_channels[i][j]]['ant_position_z']

all_lookbacks=np.zeros((len(stations),4,num_beams))
print(stations[::-1])
for i_stat,station in enumerate(stations[::-1]):
    cable_delays=all_delays[i_stat]#np.array([716.2603798064285,711.7615958304959,706.495904921158,702.195731800799])
    ant_depths=all_depths[i_stat]#np.array([-96.215,-95.174,-94.183,-93.155])

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

    beam_locs=np.linspace(np.sin(-60*np.pi/180),np.sin(60*np.pi/180),num_beams)
    beam_locs=np.arcsin(beam_locs)*180/np.pi
    #print(beam_locs)
    #print('beam locs',beam_locs)
    beam_lookback=np.zeros((4,num_beams))
    beam_lookback[0]=np.round(np.interp(beam_locs,angs,lookback[0]*int_rate))
    beam_lookback[1]=np.round(np.interp(beam_locs,angs,lookback[1]*int_rate))
    beam_lookback[2]=np.round(np.interp(beam_locs,angs,lookback[2]*int_rate))
    beam_lookback[3]=np.round(np.interp(beam_locs,angs,lookback[3]*int_rate))

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
        plt.savefig('plots/%s_arrival_times.png'%station)
        plt.close()

        plt.figure()
        plt.plot(angs,lookback[0],label='ch03')
        plt.plot(angs,lookback[1],label='ch13')
        plt.plot(angs,lookback[2],label='ch23')
        plt.plot(angs,lookback[3],label='ch33')

        plt.xlabel('angles (deg)')
        plt.ylabel('lookback (s)')
        plt.legend()
        plt.savefig('plots/%s_lookback_times.png'%station)
        plt.close()

        plt.figure()
        plt.plot(angs,lookback[0]*int_rate,label='ch0')
        plt.plot(angs,lookback[1]*int_rate,label='ch1')
        plt.plot(angs,lookback[2]*int_rate,label='ch2')
        plt.plot(angs,lookback[3]*int_rate,label='ch3')

        plt.xlabel('angles (deg)')
        plt.ylabel('lookback (int samples)')
        plt.legend()
        plt.savefig('plots/%s_lookback_interpolated_samples.png'%station)
        plt.close()

        plt.figure()
        plt.scatter(beam_locs,beam_lookback[0],label='ch0')
        plt.scatter(beam_locs,beam_lookback[1],label='ch1')
        plt.scatter(beam_locs,beam_lookback[2],label='ch2')
        plt.scatter(beam_locs,beam_lookback[3],label='ch3')

        plt.xlabel('angles (deg)')
        plt.ylabel('lookback (int samples)')
        plt.legend()
        plt.savefig('plots/%s_beam_lookback_samples.png'%station)
        plt.close()


    if print_for_quartus:
        #print('print out for quartus for station %s'%station)
        print('(',end='')
        for i in range(num_beams):
            print('(%i,%i,%i,%i)'%(beam_lookback[3][i],beam_lookback[2][i],beam_lookback[1][i],beam_lookback[0][i]),end='')
            if i==num_beams-1:
                break
            #if i==6: print()
            print(',',end='')
        print('),',end='')
        print()
    
    all_lookbacks[i_stat]=beam_lookback

for i in range(num_beams):
    break
    plt.figure()
    plt.scatter(np.linspace(0,.8,len(all_lookbacks[:,0,i])),all_lookbacks[:,0,i]-round(np.mean(all_lookbacks[:,0,i])))
    plt.scatter(1+np.linspace(0,.8,len(all_lookbacks[:,0,i])),all_lookbacks[:,1,i]-round(np.mean(all_lookbacks[:,1,i])))
    plt.scatter(2+np.linspace(0,.8,len(all_lookbacks[:,0,i])),all_lookbacks[:,2,i]-round(np.mean(all_lookbacks[:,2,i])))
    plt.scatter(3+np.linspace(0,.8,len(all_lookbacks[:,0,i])),all_lookbacks[:,3,i]-round(np.mean(all_lookbacks[:,3,i])))
    plt.xlabel('channel')
    plt.ylabel('beam %i sample delay from mean delay'%i)
    plt.xticks([0,1,2,3])
    plt.savefig('plots/station_rel_delays_beam%i.png'%i)
    plt.close()

fig,ax=plt.subplots(num_beams,1,figsize=(5,10))
for i in range(num_beams):
    ax[i].scatter(np.linspace(0,.8,len(all_lookbacks[:,0,i])),all_lookbacks[:,0,i]-np.min(all_lookbacks[:,0,i]))
    ax[i].scatter(1+np.linspace(0,.8,len(all_lookbacks[:,0,i])),all_lookbacks[:,1,i]-np.min(all_lookbacks[:,1,i]))
    ax[i].scatter(2+np.linspace(0,.8,len(all_lookbacks[:,0,i])),all_lookbacks[:,2,i]-np.min(all_lookbacks[:,2,i]))
    ax[i].scatter(3+np.linspace(0,.8,len(all_lookbacks[:,0,i])),all_lookbacks[:,3,i]-np.min(all_lookbacks[:,3,i]))
    ax[i].set_ylabel('beam %i'%i)
    ax[i].set_xticks([0,1,2,3])
fig.suptitle('Beam delays from min')
ax[num_beams-1].set_xlabel('channel')
fig.tight_layout()
plt.savefig('plots/all_beams.png')
plt.show()
plt.close()


for i in range(num_beams):
    plt.figure()
    plt.scatter(0*np.ones(len(all_lookbacks[:,0,i])),all_lookbacks[:,0,i])
    plt.scatter(1*np.ones(len(all_lookbacks[:,1,i])),all_lookbacks[:,1,i])
    plt.scatter(2*np.ones(len(all_lookbacks[:,2,i])),all_lookbacks[:,2,i])
    plt.scatter(3*np.ones(len(all_lookbacks[:,3,i])),all_lookbacks[:,3,i])
    plt.xlabel('channel')
    plt.ylabel('beam %i sample delay'%i)
    plt.savefig('plots/station_delays_beam%i.png'%i)
    plt.close()

