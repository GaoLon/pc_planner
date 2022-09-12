import numpy
import math
f = open("/home/xulong/drving_on_pc_muti_layer/data/80条轨迹_好好调参版/drive_on_pc_plan_time.txt")
lines = f.readlines()
i=0
avg_time=0
count_lines=0;
for line in lines:
	i+=1
	#print (i)
	#print (line)
	curline = line.strip().split()
	#print (curline[0])
	double_curline=float(curline[0])
	avg_time+=double_curline
	count_lines+=1
	print("now_avg_time")
	print(avg_time/count_lines)
print("finish_all",avg_time)
print("count_lines ",count_lines)
print(avg_time/count_lines)
f.close()
