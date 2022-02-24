#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
if string.find(pwd,"Run") then  dofile('../include.lua')
else    dofile('./include.lua') end

require'wcm'
local unix=require'unix'
local ffi = require'ffi'
local vector=require'vector'
local util=require'util'
local T = require'Transform'

local rospub=require'rospub2'
local Body  = require'Body'
local K=Body.Kinematics

-- rospub.init_custom('pub_log',{"/depth_image_new","/CAM/camera/image"},{},{"/path"},{"/lowres_map"})
rospub.init_custom('pub_log',{"/CAM/range_finder/range_image","/CAM/camera/image"},{},{"/path"},{"/lowres_map"})

local jointnames={
	"shoulder_pan_joint",
	"shoulder_lift_joint",
	"elbow_joint",
	"wrist_1_joint",
	"wrist_2_joint",
	"wrist_3_joint"
}

local jangle0=vector.new({0, -45, 60, -15+90,90,0})*DEG_TO_RAD
local jangle1=vector.new({0, 70, 20, -0,-0,0})*DEG_TO_RAD --facking right (from robot)
--



-- local p_can1={-0.4,-0.8,0.20,180*DEG_TO_RAD,0,180*DEG_TO_RAD}
-- local p_can1={-0.65,-0.4,0.20,180*DEG_TO_RAD,0,180*DEG_TO_RAD}
local p_can1={-0.65,0,0.20,180*DEG_TO_RAD,0,180*DEG_TO_RAD}
local p_can1={-0.65,0.4,0.20,180*DEG_TO_RAD,0,180*DEG_TO_RAD}
local p_can1={-0.4,0.8,0.20,180*DEG_TO_RAD,0,180*DEG_TO_RAD}
local p_can1={-0.4,0.8,0.10,180*DEG_TO_RAD,90*DEG_TO_RAD,180*DEG_TO_RAD}


local q_can1=vector.new(K.inverse_arm(p_can1, jangle1,2))






local side_move=0.10
local down_move=0.20



local p0=K.forward_arm(jangle1)
print("p0:",unpack(p0))


local px,py,p_yaw=0.66,0,90*DEG_TO_RAD --gripper towards left
local px,py,p_yaw=0.66,0,45*DEG_TO_RAD
local px,py,p_yaw=0.66,0,-135*DEG_TO_RAD



local px,py,p_yaw=0.60,0.57,-90*DEG_TO_RAD --gripper towards right

local px,py,p_yaw=0.435,0.499,-90*DEG_TO_RAD --gripper towards right



local p1={px-side_move*math.cos(p_yaw),py-side_move*math.sin(p_yaw),0.27,     3.141592, 0, p_yaw} --gripper facing backward (from robot)

local q1=vector.new(K.inverse_arm(p1, jangle1,2))
print("Q1:",unpack(q1/DEG_TO_RAD))


local p15=util.shallow_copy(p1);p15[3]=p15[3]-0.20
local q15=vector.new(K.inverse_arm(p15, jangle1,2))
print("Q15:",unpack(q15/DEG_TO_RAD))

--q1 wrist: (-23, -162, 0)




local function jangle_interpolate(j0, j1, totaltime, div)
	local jangle2,dur2={},{}
	local count=0
	for i=0,div do
		local ph=i/div
		local q1 = ph*j1 + (1-ph)*j0
		jangle2[count*6+1],jangle2[count*6+2],jangle2[count*6+3],jangle2[count*6+4],jangle2[count*6+5],jangle2[count*6+6]=
			q1[1],q1[2]-90*DEG_TO_RAD,q1[3],q1[4],q1[5],q1[6]		--90 degree offset between our IK and actual joint angle
		dur2[count+1]=i*totaltime/div
		count=count+1
	end
	return jangle2,dur2
end

local function IK_interpolate(j0, xyz, totaltime, div)
	local jangle2,dur2={},{}
	local count=0
	local p0=K.forward_arm(j0)
	local q1
	for i=0,div do
		local ph=i/div
		local p1={p0[1]+ph*xyz[1],p0[2]+ph*xyz[2],p0[3]+ph*xyz[3],p0[4],p0[5],p0[6]}
		q1=vector.new(K.inverse_arm(p1, j0,2))
		jangle2[count*6+1],jangle2[count*6+2],jangle2[count*6+3],jangle2[count*6+4],jangle2[count*6+5],jangle2[count*6+6]=
			q1[1],q1[2]-90*DEG_TO_RAD,q1[3],q1[4],q1[5],q1[6]		--90 degree offset between our IK and actual joint angle
		dur2[count+1]=i*totaltime/div
		count=count+1
	end
	return jangle2,dur2,q1
end

local function traj_concat(t1,t2,dur1,dur2,int)
	local j3,dur3={},{}
	if not int then int=0 end
	for i=1,#t1 do j3[i]=t1[i] end
	for i=1,#t2 do j3[i+#t1]=t2[i] end
	for i=1,#dur1 do dur3[i]=dur1[i] end
	for i=1,#dur2 do dur3[i+#dur1]=dur2[i]+dur1[#dur1]+int end
	return j3,dur3
end



local jtraj1, dur1=jangle_interpolate(jangle0, q1, 1, 100)
local jtraj2, dur2,q2_fin=IK_interpolate(q1, {0,0,-0.22}, 1, 100)
local jtraj22,dur22,q2_fin=IK_interpolate(q2_fin, {side_move*math.cos(p_yaw),side_move*math.sin(p_yaw),0}, 1, 100)


local jtrajstop, durstop=jangle_interpolate(q2_fin, q2_fin, 2, 100)


local jtraj3, dur3,q3_fin=IK_interpolate(q2_fin, {0,0,0.22}, 1, 100) --LIFT
local jtraj4, dur4=jangle_interpolate(q3_fin, jangle0, 1, 100) --ZERO POS
local jtraj5, dur5=jangle_interpolate(jangle0, q_can1, 1, 100) --CAN POS


local jtraj12, dur12=traj_concat(jtraj1, jtraj2, dur1, dur2)
local jtraj12, dur12=traj_concat(jtraj12, jtraj22, dur12, dur22)


local jtraj34, dur34=traj_concat(jtraj3, jtraj4, dur3, dur4)
local jtraj34, dur34=traj_concat(jtraj34, jtraj5, dur34, dur5)





local jtraj1234, dur1234=traj_concat(jtraj12, jtrajstop, dur12, durstop)
local jtraj1234, dur1234=traj_concat(jtraj1234, jtraj34, dur1234, dur34)

-- local jtraj2, dur2=jangle_interpolate(q2, q1, 1, 15)
-- local jtraj3, dur3=jangle_interpolate(q3, q4, 5, 10)
-- local jtraj4, dur4=jangle_interpolate(q4, q1, 5, 10)

local gripperjointnames={
	"finger_1_joint_1",
	"finger_2_joint_1",
	"finger_middle_joint_1"
}

--

local grip0=vector.ones(6)*0.1 --full open
local grip1=vector.ones(6)*1.1 --full close
-- local grip1=vector.ones(6)*0.8 --semi close

local running=true

print("Sending...")

rospub.jointactiongoal(gripperjointnames, "test2",grip0, {0.5,1},1)
unix.usleep(1e6*1)

-- rospub.jointactiongoal(jointnames, "test1",jtraj12, dur12,0)
rospub.jointactiongoal(jointnames, "test1",jtraj1234, dur1234,0)
-- rospub.jointactiongoal(jointnames, "test1",jtraj1, dur1,0)
unix.usleep(1e6*3)
rospub.jointactiongoal(gripperjointnames, "test2",grip1, {0.5,1},1)

unix.usleep(1e6*6)
rospub.jointactiongoal(gripperjointnames, "test2",grip0, {0.5,1},1)
