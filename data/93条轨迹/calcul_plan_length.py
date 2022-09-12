import numpy
import math
f = open("/home/xulong/drving_on_pc_muti_layer/data/93条轨迹/drive_on_pc_traj_length.txt")
lines = f.readlines()
i=0
avg_length=0
count_lines=0;
for line in lines:
	i+=1
	#print (i)
	#print (line)
	curline = line.strip().split()
	#print (curline[0])
	double_curline=float(curline[0])
	avg_length+=double_curline
	count_lines+=1
	print("now_avg_length")
	print(avg_length/count_lines)
print("finish_all",avg_length)
print("count_lines ",count_lines)
print(avg_length/count_lines)
f.close()
