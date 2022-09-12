import numpy
import math
f = open("/home/xulong/driving_on_pc_one_layer/driving_on_pc/data/drive_on_pc_traj.txt")
lines = f.readlines()
i=0
avg_angle_rad_all_lines=0
count_lines=0;
for line in lines:
	i+=1
	#print (i)
	#print (line)
	curline = line.strip().split()
	#print (curline[0])
	double_curline=float(curline[0])
	size=len(curline)/8
	avg_cur=0;
	count=0;
	for i in range (int(size)-2):
		point1=numpy.array([float(curline[i*8+0]),float(curline[i*8+1])])
		point2=numpy.array([float(curline[(i+1)*8+0]),float(curline[(i+1)*8+1])])
		point3=numpy.array([float(curline[(i+2)*8+0]),float(curline[(i+2)*8+1])])
		vector1=point2-point1
		vector2=point3-point2
		if(numpy.linalg.norm(vector1)==0):
			print("error vector1")
			continue;
		if(numpy.linalg.norm(vector2)==0):
			print("error vector2")
			continue;
		cos_value=vector1.dot(vector2)/(numpy.linalg.norm(vector1)*numpy.linalg.norm(vector2))
		if(cos_value>1):
			cos_value=1
		if(cos_value<-1):
			cos_value=-1
		angle_rad=math.acos(cos_value)
		dist=numpy.linalg.norm(vector1)
		if(dist==0):
			print("error")
			continue;
		avg_cur+=abs(angle_rad)/dist
		count+=1
	
	#print(avg_cur/count)
	avg_angle_rad_all_lines+=avg_cur/count
	count_lines+=1
	print("now_avg_all")
	print(avg_angle_rad_all_lines/count_lines)
print("finish_all")
print("count_lines ",count_lines)
print(avg_angle_rad_all_lines/count_lines)
f.close()
