import numpy as np

thresholds=np.ones(12,dtype=int)*500
#thresholds=np.array([200,200,200,200,1,1,1,1,1,1,1,1])

f=open("data/input_thresholds.txt","w")
for i in range(12):
    f.write(f"{thresholds[-i]:012b} ")
f.close()