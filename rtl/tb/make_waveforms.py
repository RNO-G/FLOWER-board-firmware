import numpy as np

#nuradiomc dah dah dah

ch0_data=np.random.randint(-10,10,size=1024)+128
ch1_data=np.random.randint(-10,10,size=1024)+128
ch2_data=np.random.randint(-10,10,size=1024)+128
ch3_data=np.random.randint(-10,10,size=1024)+128


#ch0_data=np.zeros(1024,dtype=int)+128
#ch1_data=np.zeros(1024,dtype=int)+128
#ch2_data=np.zeros(1024,dtype=int)+128
#ch3_data=np.zeros(1024,dtype=int)+128

ch3_data[199]=32+128
ch3_data[200]=32+128
ch3_data[201]=-32+128
ch3_data[202]=-16+128

#ch0_data[501]=-32+128


print(ch0_data[0])

ch0_cond=ch0_data.reshape((256,4))
ch1_cond=ch1_data.reshape((256,4))
ch2_cond=ch2_data.reshape((256,4))
ch3_cond=ch3_data.reshape((256,4))

ch0_vals=np.zeros(256,dtype=int)
ch1_vals=np.zeros(256,dtype=int)
ch2_vals=np.zeros(256,dtype=int)
ch3_vals=np.zeros(256,dtype=int)

#order gets flipped going into vhdl modules... here is [0, 1, 2, ..., 30, 31] but in fpga land its [31,30,...,2,1,0]
for i in range(256):
    ch0_vals[i]=(ch0_cond[i][3])+(ch0_cond[i][2]<<8)+(ch0_cond[i][1]<<16)+(ch0_cond[i][0]<<24)
    ch1_vals[i]=(ch1_cond[i][3])+(ch1_cond[i][2]<<8)+(ch1_cond[i][1]<<16)+(ch1_cond[i][0]<<24)
    ch2_vals[i]=(ch2_cond[i][3])+(ch2_cond[i][2]<<8)+(ch2_cond[i][1]<<16)+(ch2_cond[i][0]<<24)
    ch3_vals[i]=(ch3_cond[i][3])+(ch3_cond[i][2]<<8)+(ch3_cond[i][1]<<16)+(ch3_cond[i][0]<<24)

print((ch0_vals[0]&0xff))

f=open("data/input_waveforms.txt",mode="w")
for i in range(256):
    if i==256-1:
        f.write(f"{ch0_vals[i]:032b} {ch1_vals[i]:032b} {ch2_vals[i]:032b} {ch3_vals[i]:032b}")
    else:
        f.write(f"{ch0_vals[i]:032b} {ch1_vals[i]:032b} {ch2_vals[i]:032b} {ch3_vals[i]:032b}\n")
f.close()

#for easier plotting
np.savetxt("data/plot_input_waveforms.txt",(ch0_data-128,ch1_data-128,ch2_data-128,ch3_data-128))
