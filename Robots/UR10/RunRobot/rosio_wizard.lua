#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
if string.find(pwd,"Run") then  dofile('../include.lua')
else    dofile('./include.lua') end

require'wcm'
require'hcm'

local unix=require'unix'
local ffi = require'ffi'
local vector=require'vector'
local util=require'util'
local T = require'Transform'

local rossub=require'rossub'
local rospub=require'rospub2'
local Body  = require'Body'
local K=Body.Kinematics

-- rospub.init_custom('pub_log',{"/depth_image_new","/CAM/camera/image"},{},{"/path"},{"/lowres_map"})
rossub.init('rosio_wizard')
rospub.init_custom('rosio_wizard',{"/CAM/range_finder/range_image","/CAM/camera/image"},{},{"/path"},{"/lowres_map"})

local sub_idx_jointstate=rossub.subscribeJointState('/joint_states')
local sub_idx_gripperjointstate=rossub.subscribeJointState('/gripper_joint_states')

local jointnames={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"}
local gripperjointnames={"finger_1_joint_1","finger_2_joint_1","finger_middle_joint_1"}

local cur_q={0,0,0,0,0,0}
local grip_fo=vector.ones(6)*0.1 --full release

-- local grip1=vector.ones(6)*0.8 --semi close





ang_vel1=60*DEG_TO_RAD --works well

ang_vel1=90*DEG_TO_RAD



linear_vel=0.20
local down_move=0.40
local pickup_timing={0.5,0.5, 1, 1}--down side hold up
-- K.setup_tool_param(0.13,0)
-- local side_move,down_move=0.08,0.40
local z_height0=0.07
local q_last=nil

--1 Coke can: r=0.03
--2 Beer cab
--3 Coffee: r=0.051
--4  Milk small: 0.027 by 0.036
--5  Wine: r=0.037
--6 Watter: r=0.04
--7 Coke bottle: r=0.02
--8 Milk: 0.08 by 0.08
--9 Beer
--10 Snack
--11 Tissue: r=0.0152*3
--12 Painkiller: r= 0.0216



-- Beer: r=0.02

local function sig(k,x)
	local ret=(k*x-x)/(2*k*x-k-1)
	return ret
end

-- for i=0,1.02,0.02 do
-- 	local c=0.8
-- 	if i<0.5 then y=sig(c,2*i)*0.5
-- 	else y=0.5*sig(-c,(2*(i-0.5)) )+0.5 end
-- 	print( string.format("x:%.2f y:%.2f",i, y ))
-- end

local function jangle_interpolate(j0, j1, totaltime, div, c)
	local jangle2,dur2={},{}
	local count=0
	for i=0,div do
		local ph,ph1=i/div,i/div
		if c then
			if ph<0.5 then ph1=sig(c,ph*2)*0.5
			else ph1=0.5*sig(-c,(2*(ph-0.5)) )+0.5 end
		end
		local q1 = ph1*j1 + (1-ph1)*j0
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
	local j1_old,j5_old=j0[1],j0[5]
	for i=0,div do
		local ph=i/div
		local p1={p0[1]+ph*xyz[1],p0[2]+ph*xyz[2],p0[3]+ph*xyz[3],p0[4],p0[5],p0[6]}
		q1=vector.new(K.inverse_arm(p1, j0,5))

		--Wrist 360 deg jump prevention
		if j5_old then
			if q1[5]<j5_old-180*DEG_TO_RAD then q1[5]=q1[5]+360*DEG_TO_RAD end
			if q1[5]>j5_old+180*DEG_TO_RAD then q1[5]=q1[5]-360*DEG_TO_RAD end
		end
		j5_old=q1[5]

		if j1_old then
			if q1[1]<j1_old-180*DEG_TO_RAD then q1[1]=q1[1]+360*DEG_TO_RAD end
			if q1[1]>j1_old+180*DEG_TO_RAD then q1[1]=q1[1]-360*DEG_TO_RAD end
		end
		j1_old=q1[1]


		jangle2[count*6+1],jangle2[count*6+2],jangle2[count*6+3],jangle2[count*6+4],jangle2[count*6+5],jangle2[count*6+6]=
			q1[1],q1[2]-90*DEG_TO_RAD,q1[3],q1[4],q1[5],q1[6]		--90 degree offset between our IK and actual joint angle
		dur2[count+1]=i*totaltime/div
		count=count+1
	end
	return jangle2,dur2,q1
end

local function traj_concat(t1,t2,dur1,dur2)
	local j3,dur3={},{}
	for i=1,#t1 do j3[i]=t1[i] end
	for i=1,#t2 do j3[i+#t1]=t2[i] end
	for i=1,#dur1 do dur3[i]=dur1[i] end
	for i=1,#dur2 do dur3[i+#dur1]=dur2[i]+dur1[#dur1] end
	return j3,dur3
end


local function update_jointstate()
	local ret1=rossub.checkJointState(sub_idx_jointstate)
	local ret2=rossub.checkJointState(sub_idx_gripperjointstate)
	if ret1 then
		cur_q[1],cur_q[2],cur_q[3],cur_q[4],cur_q[5],cur_q[6]=
			ret1[1],ret1[2]+90*DEG_TO_RAD,ret1[3],ret1[4],ret1[5],ret1[6] --90 degree offset conversion
	end
end

local function check_angle_err(q1,q2)
	local err=0
	for i=1,#q1 do err=err+math.abs( util.mod_angle(q1[i]-q2[i]) ) end
	return err
end

local function check_angletime(q1,q2, t_min, avel_lim)
	local err=0
	for i=1,#q1 do err=math.max(err, math.abs( q1[i]-q2[i] )/avel_lim   ) end
	return math.max(t_min,err)
end



local last_motion_execute=0

local jtraj_all, dur_all
local t_motion_start,t_grip, t_release,t_end
local q_grip, q_release

local motion_stage=0
hcm.set_motionplan_execute(0)
local motion_type=0


local function gen_motion(targetxya, tool_param_x,side_move, down_move, p_can, gripopen_target, grip_target)
	local k1=0.8
	local k2=0.3
	local t_drop=1.5
	local t_post_release=1
	local jangle0=vector.new({0, 0, 75, 15,90,0})*DEG_TO_RAD  --middle joint angle point
	local jangle1=vector.new({0, 70, 20, -0,-0,0})*DEG_TO_RAD --reference arm angle for IK calculation
	K.setup_tool_param(tool_param_x,0)--how far away the hand should be
	grip0,grip1=vector.ones(6)*gripopen_target,vector.ones(6)*grip_target

	--pose on the target object
	local p1={targetxya[1]-side_move*math.cos(targetxya[3]),targetxya[2]-side_move*math.sin(targetxya[3]),
					z_height0+down_move,3.141592, 0*DEG_TO_RAD, targetxya[3] } --gripper facing backward (from robot)
	local q_pre_pickup=vector.new(K.inverse_arm(p1, jangle1,5))

	local t_init=0.5
	if q_last then t_init=check_angletime(q_last, jangle0, 0.5, ang_vel1)
	else q_last=vector.new(jangle0) end

	local q_init=vector.new(util.shallow_copy(jangle0));
	q_init[1],q_init[5]=q_pre_pickup[1],q_pre_pickup[5]

	local q_can=vector.new(K.inverse_arm(p_can, jangle1,5));q_can[6]=0 --zero wrist spin
	if q_can[1]<0 then q_can[1]=q_can[1]+360*DEG_TO_RAD end --only rotate left (prevent item hitting)

	--move: q_last -> q_init -> q_pre_pickup -> q_pickup1(IK) -> q_pickup2(IK) -> q_post_pickup(IK) -> q_can_high -> q_can

	local t_pre_pickup=check_angletime(q_init, q_pre_pickup, 0.5, ang_vel1)
	local jtraj0, dur0=jangle_interpolate(q_last,q_init, t_init, 100)

	local jtraj1, dur1=jangle_interpolate(q_init, q_pre_pickup, t_pre_pickup, 100,k1)
	local jtraj2, dur2,q_pickup1=IK_interpolate(q_pre_pickup, {0,0,-down_move}, pickup_timing[1], 100)
	local jtraj22,dur22,q_pickup2=IK_interpolate(q_pickup1, {side_move*math.cos(targetxya[3]),side_move*math.sin(targetxya[3]),0}, pickup_timing[2], 100)
	q_grip=util.shallow_copy(q_pickup2) --store this for arm arrival check

	local jtrajstop, durstop=jangle_interpolate(q_pickup2, q_pickup2, pickup_timing[3], 100) --wait during grasping
	local jtraj3, dur3,q_post_pickup=IK_interpolate(q_pickup2, {0,0,down_move}, pickup_timing[4], 100) --raise up vertically

	local q_can_high=vector.new(util.shallow_copy(q_post_pickup)); q_can_high[1]=q_can[1] --rotate horizontally to the target shoulder yaw
	local t_can_high=check_angletime(q_post_pickup, q_can_high, 0.5, ang_vel1)

	--PREVENT overturning of the 5th joint
	if q_can[5]>q_can_high[5]+180*DEG_TO_RAD then q_can[5]=q_can[5]-360*DEG_TO_RAD end
	if q_can[5]<q_can_high[5]-180*DEG_TO_RAD then q_can[5]=q_can[5]+360*DEG_TO_RAD end

	local jtraj4, dur4=jangle_interpolate(q_post_pickup, q_can_high, t_can_high, 100,k2)
	local jtraj5, dur5=jangle_interpolate(q_can_high, q_can, t_drop, 100)
	q_release=util.shallow_copy(q_can) --store this for arm arrival check
	q_last=vector.new(util.shallow_copy(q_can)) --store this for next arm movement

	jtraj_all,dur_all=traj_concat(jtraj0, jtraj1, dur0, dur1)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj2, dur_all, dur2)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj22, dur_all, dur22)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtrajstop, dur_all, durstop)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj3, dur_all, dur3)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj4, dur_all, dur4)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj5, dur_all, dur5)
	local t=unix.time()

	t_grip= t + t_init + t_pre_pickup + pickup_timing[1]+pickup_timing[2]
	t_release= t_grip + pickup_timing[3]+pickup_timing[4]+t_can_high+t_drop
	t_end = t_release+ t_post_release
	motion_stage=1


	-- print("DURALL:",#dur_all, dur_all[1])


	rospub.jointactiongoal(jointnames, "test1",jtraj_all, dur_all,0)
	rospub.jointactiongoal(gripperjointnames, "test2",grip0, {0.5,1},1)
end


local function gen_motion2(targetxya, tool_param_x,side_move, down_move, p_can, gripopen_target, grip_target)
	local k1=0.8
	local k2=0.3
	local t_drop=1.5
	local t_post_release=1
	local motion_div=100

	local jangle0=vector.new({180, 0, 75, 15,90,0})*DEG_TO_RAD  --middle joint angle point
	local jangle1=vector.new({0, 70, 20, -0,-0,0})*DEG_TO_RAD --reference arm angle for IK calculation
	K.setup_tool_param(tool_param_x,0)--how far away the hand should be
	grip0,grip1=vector.ones(6)*gripopen_target,vector.ones(6)*grip_target

	--pose on the target object
	local p1={targetxya[1]-side_move*math.cos(targetxya[3]),targetxya[2]-side_move*math.sin(targetxya[3]),
					z_height0+down_move,3.141592, 0*DEG_TO_RAD, targetxya[3] } --gripper facing backward (from robot)
	local q_pre_pickup=vector.new(K.inverse_arm(p1, jangle1,5))

	local t_init=0.5
	if q_last then t_init=check_angletime(q_last, jangle0, 0.5, ang_vel1)
	else q_last=vector.new(util.shallow_copy(cur_q)) end

	local q_init=vector.new(util.shallow_copy(jangle0));
	q_init[1],q_init[5]=q_pre_pickup[1],q_pre_pickup[5]

	local q_can=vector.new(K.inverse_arm(p_can, jangle1,5));q_can[6]=0 --zero wrist spin
	if q_can[1]<0 then q_can[1]=q_can[1]+360*DEG_TO_RAD end --only rotate left (prevent item hitting)

	--move: q_last -> q_init -> q_pre_pickup -> q_pickup1(IK) -> q_pickup2(IK) -> q_post_pickup(IK) -> q_can_high -> q_can

	local t_pre_pickup=check_angletime(q_init, q_pre_pickup, 0.5, ang_vel1)
	local jtraj0, dur0=jangle_interpolate(q_last,q_init, t_init, motion_div)

	local jtraj1, dur1=jangle_interpolate(q_init, q_pre_pickup, t_pre_pickup, motion_div)
	local jtraj2, dur2,q_pickup1=IK_interpolate(q_pre_pickup, {0,0,-down_move}, pickup_timing[1], 100)
	local jtraj22,dur22,q_pickup2=IK_interpolate(q_pickup1, {side_move*math.cos(targetxya[3]),side_move*math.sin(targetxya[3]),0}, pickup_timing[2], 100)
	q_grip=util.shallow_copy(q_pickup2) --store this for arm arrival check

	local jtrajstop, durstop=jangle_interpolate(q_pickup2, q_pickup2, pickup_timing[3], 100) --wait during grasping
	local jtraj3, dur3,q_post_pickup=IK_interpolate(q_pickup2, {0,0,down_move}, pickup_timing[4], 100) --raise up vertically

	local q_can_high=vector.new(util.shallow_copy(q_post_pickup)); q_can_high[1]=q_can[1] --rotate horizontally to the target shoulder yaw
	local t_can_high=check_angletime(q_post_pickup, q_can_high, 0.5, ang_vel1)

	--PREVENT overturning of the 5th joint
	if q_can[5]>q_can_high[5]+180*DEG_TO_RAD then q_can[5]=q_can[5]-360*DEG_TO_RAD end
	if q_can[5]<q_can_high[5]-180*DEG_TO_RAD then q_can[5]=q_can[5]+360*DEG_TO_RAD end

	local jtraj4, dur4=jangle_interpolate(q_post_pickup, q_can_high, t_can_high, motion_div)
	local jtraj5, dur5=jangle_interpolate(q_can_high, q_can, t_drop, motion_div)
	local jtraj55, dur55=jangle_interpolate(q_can, q_can, 0.5, motion_div)
	local jtraj6, dur6 , q_can_raise=IK_interpolate(q_can, {0,0,0.30}, 0.5, 100) --raise up vertically

	q_last=vector.new(util.shallow_copy(q_can_raise))
	q_release=util.shallow_copy(q_can) --store this for arm arrival check

	jtraj_all,dur_all=traj_concat(jtraj0, jtraj1, dur0, dur1)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj2, dur_all, dur2)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj22, dur_all, dur22)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtrajstop, dur_all, durstop)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj3, dur_all, dur3)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj4, dur_all, dur4)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj5, dur_all, dur5)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj55, dur_all, dur55)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj6, dur_all, dur6) --ADDED
	local t=unix.time()

	t_grip= t + t_init + t_pre_pickup + pickup_timing[1]+pickup_timing[2]
	t_release= t_grip + pickup_timing[3]+pickup_timing[4]+t_can_high+t_drop
	t_end = t_release+ t_post_release+1
	motion_stage=1

	rospub.jointactiongoal(jointnames, "test1",jtraj_all, dur_all,0)
	rospub.jointactiongoal(gripperjointnames, "test2",grip0, {0.5,1},1)
end

local function update_motionplan()
	motion_type=0
	local targettype=hcm.get_motionplan_targettype()
	local targetsize=hcm.get_motionplan_targetsize()
	local targetxya=hcm.get_motionplan_targetxya()
	local targetid=hcm.get_motionplan_targetid()
	local sel=wcm.get_objects_selected()
	local side_move,down_move=0.08,0.40
	print(string.format("MOTION EXECUTE, Target:(%.4f %.4f %.1f) id:%d size:%.3f type:%d",targetxya[1],targetxya[2],targetxya[3]/DEG_TO_RAD,targetid, targetsize, targettype ))

	local tool_param_delta,grip_target,gripopen_target=0,hcm.get_motionplan_graspwidth(),0.2
	if targetsize>0.10 then targetsize=0.10 end
	print("Item radius::::",targetsize/2)
	if targetsize/2>0.04 then
		tool_param_delta=-0.01
		gripopen_target,grip_target=0.2,0.7 --full open, semi grab
	else
		tool_param_delta=-0.03 --closer
		gripopen_target,grip_target=0.3,1.1 --semi open, full grab
	end

	if targetid==1 then tool_param_delta,grip_target,gripopen_target=-0.01,0.9,0.3 -- coke can
	elseif targetid==2 then tool_param_delta,grip_target,gripopen_target=-0.01,0.9,0.3 --beer can
	elseif targetid==3 then tool_param_delta,grip_target,gripopen_target=-0.01,0.7,0.1 --coffee
	elseif targetid==4 then tool_param_delta,grip_target,gripopen_target=-0.02,1.1,0.3 --small milk
	elseif targetid==5 then tool_param_delta,grip_target,gripopen_target=-0.03,0.7,0.1 --WINE
	elseif targetid==6 then tool_param_delta,grip_target,gripopen_target=-0.03,0.7,0.1 --WATER
	elseif targetid==7 then tool_param_delta,grip_target,gripopen_target=-0.02,1.1,0.3  --coke bottle
	elseif targetid==8 then tool_param_delta,grip_target,gripopen_target=-0.03,0.7,0.1 --milk pack
	elseif targetid==9 then tool_param_delta,grip_target, side_move = -0.02, 1.1, 0.08 --beer
	elseif targetid==12 then tool_param_delta,grip_target=-0.02,1.1 --small milk
	end

	local p_can={-0.4,-0.8,0.10,180*DEG_TO_RAD,90*DEG_TO_RAD,180*DEG_TO_RAD} --plastic
	if targettype==1 then p_can[1],p_can[2]=-0.65, 0 -- plastic
	elseif targettype==2 then p_can[1],p_can[2]=-0.4, -0.8 -- paper
	elseif targettype==3 then p_can[1],p_can[2]=-0.65, -0.4 --can
	elseif targettype==4 then p_can[1],p_can[2]=-0.4, 0.8 --vinyl
	elseif targettype==5 then p_can[1],p_can[2]=-0.65, 0.4 --bottle
	end

	--testing with zero side move
	--beer stck
	tool_param_delta=0.03
	side_move=0
	if targetid==3 then grip_target=0.5 end --coffee
	if targetid==9 then grip_target=0.9 end --beer





	p_can[3]=0.03 --lower
	--gen_motion(targetxya,0.13+tool_param_delta,side_move,down_move,p_can,gripopen_target,grip_target)
	gen_motion2(targetxya,0.13+tool_param_delta,side_move,down_move,p_can,gripopen_target,grip_target)
end


local function update_motionplan2() --for vertical pickup
	motion_type=1
	local motion_div=100

	local targettype=hcm.get_motionplan_targettype()
	local targetsize=hcm.get_motionplan_targetsize()
	local targetxya=hcm.get_motionplan_targetxya()
	local objsel=wcm.get_objects_selected()
	local ominw=wcm.get_objects_min_width()
	local omaxw=wcm.get_objects_max_width()
	print(string.format("MOTION EXECUTE, Target:%.3f %.3f (%.1f)",targetxya[1],targetxya[2],targetxya[3]/DEG_TO_RAD))

	local gripopen_target,grip_target=0.2,0.7 --full open, semi grab for snack
	grip0,grip1=vector.ones(6)*gripopen_target,vector.ones(6)*grip_target
	local tool_param_x=0.13
	K.setup_tool_param(tool_param_x,0)--how far away the hand should be
	local targetid=hcm.get_motionplan_targetid()
	local z_height_vg=0.06
	-- if targetid==10 then z_height_vg=0.05	end

	local jangle0=vector.new({0, 0, 60, -60,0,0})*DEG_TO_RAD
	local jangle1=vector.new({0, 70, 20, -0,-0,0})*DEG_TO_RAD --facking right (from robot)
	local p_can={-0.4,-0.8,0.20,180*DEG_TO_RAD,90*DEG_TO_RAD,180*DEG_TO_RAD}
	if targettype==1 then p_can[1],p_can[2]=-0.65, 0 -- plastic
	elseif targettype==2 then p_can[1],p_can[2]=-0.4, -0.8 -- paper
	elseif targettype==3 then p_can[1],p_can[2]=-0.65, -0.4 --can
	elseif targettype==4 then p_can[1],p_can[2]=-0.4, 0.8 --vinyl
	elseif targettype==5 then p_can[1],p_can[2]=-0.65, 0.4 --bottle
	end

	--move: q_last -> q_init -> q_pre_pickup -> q_pickup(IK) -> q_post_pickup(IK) -> q_can_high -> q_can

	--initialization pose
	if not q_last then q_last=vector.new(util.shallow_copy(cur_q) ) end

	--pose on the target object
	local p1={targetxya[1],targetxya[2],z_height_vg+down_move,3.141592, 90*DEG_TO_RAD, targetxya[3] } --gripper facing backward (from robot)
	local q_pre_pickup=vector.new(K.inverse_arm(p1, jangle1,5))
	if q_pre_pickup[6]>90*DEG_TO_RAD then q_pre_pickup[6]=q_pre_pickup[6]-180*DEG_TO_RAD end
	if q_pre_pickup[6]<-90*DEG_TO_RAD then q_pre_pickup[6]=q_pre_pickup[6]+180*DEG_TO_RAD end

	--init pose
	local q_init=vector.new(util.shallow_copy(jangle0));
	-- q_init[1],q_init[5]=q_pre_pickup[1],q_pre_pickup[5]
	q_init[1],q_init[5],q_init[6]=q_pre_pickup[1],q_pre_pickup[5],q_pre_pickup[6]
	local t_init=check_angletime(q_last, q_init, 0.5, ang_vel1)
	local jtraj0, dur0=jangle_interpolate(q_last, q_init, t_init, 100)

	local t_pre_pickup=check_angletime(q_init, q_pre_pickup, 0.5, ang_vel1)
	local jtraj1, dur1=jangle_interpolate(q_init, q_pre_pickup, t_pre_pickup, 100)


	-- local t_down=1
	local t_down=0.7
	local t_down=1.5

	local jtraj2, dur2,q2_fin=IK_interpolate(q_pre_pickup, {0,0,-down_move}, t_down, 100)
	local jtrajstop, durstop=jangle_interpolate(q2_fin, q2_fin, pickup_timing[3], 100)
	local jtraj3, dur3,q_post_pickup=IK_interpolate(q2_fin, {0,0,down_move}, pickup_timing[4], 100) --raise up
	q_grip=util.shallow_copy(q2_fin)

	--target box pose
	local q_can=vector.new(K.inverse_arm(p_can, jangle1,5))
	if q_can[6]>90*DEG_TO_RAD then q_can[6]=q_can[6]-180*DEG_TO_RAD end
	if q_can[6]<-90*DEG_TO_RAD then q_can[6]=q_can[6]+180*DEG_TO_RAD end
	if q_can[1]<0 then q_can[1]=q_can[1]+360*DEG_TO_RAD end --only rotate left (prevent item hitting)

	-- local q_can_high=vector.new({q_can[1],q3_fin[2],q3_fin[3],q3_fin[4],q3_fin[5],q3_fin[6]}) --target location

	local t_can=check_angletime(q_post_pickup, q_can, 0.5, ang_vel1)
	local jtraj4, dur4=jangle_interpolate(q_post_pickup, q_can, t_can, 100)

	local downpush={0.10,0,-0.40}
	local uppush={-0.10,0,0.40}

	local jtraj6, dur6,q5_fin=IK_interpolate(q_can, downpush, 1, 100) --raise up
	local jtraj7, dur7,q6_fin=IK_interpolate(q5_fin, uppush, 1, 100) --raise up
	q_release=util.shallow_copy(q_can)

--
-- print(unpack(q_init))
-- print(unpack(q_pre_pickup))

	jtraj_all,dur_all=traj_concat(jtraj0, jtraj1, dur0, dur1)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj2, dur_all, dur2)

	jtraj_all,dur_all=traj_concat(jtraj_all, jtrajstop, dur_all, durstop)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj3, dur_all, dur3)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj4, dur_all, dur4)
	-- jtraj_all,dur_all=traj_concat(jtraj_all, jtraj5, dur_all, dur5)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj6, dur_all, dur6) --down
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj7, dur_all, dur7) --up

	q_last=vector.new(util.shallow_copy(q6_fin))

	local t=unix.time()
	t_grip= t + t_init + t_pre_pickup + t_down
	t_release= t_grip + pickup_timing[3]+pickup_timing[4]+t_can+1
	t_end = t_release+ 2
	motion_stage=1

	rospub.jointactiongoal(jointnames, "test1",jtraj_all, dur_all,0)
	rospub.jointactiongoal(gripperjointnames, "test2",grip_fo, {0.5,1},1)
end

local function check_motionplan()
	local motion_execute=hcm.get_motionplan_execute()
	if motion_execute>last_motion_execute then
		last_motion_execute=motion_execute
		local objsel=wcm.get_objects_selected()
		local grasptype=hcm.get_motionplan_grasptype()
		if objsel>0 then
			if grasptype==0 then
				update_motionplan()
			else
				update_motionplan2()
			end
		else
			print("NO ITEM CAN BE MOVED")
			last_motion_execute=0
			hcm.set_motionplan_execute(0)
		end

	end
end

local t_timeout1=5
local t_timeout2=5

local function motioncontrol()
	local t=unix.time()
	if motion_stage==1 then
		local angle_err=check_angle_err(cur_q,q_grip)
		-- if (angle_err<10*DEG_TO_RAD)then print("Angle err:" , angle_err/DEG_TO_RAD) end
		if t>t_grip then
			if angle_err<5*DEG_TO_RAD then
				print("Approach time:",t-t_grip)
				rospub.jointactiongoal(gripperjointnames, "test2",grip1, {0.5,1},1)
				motion_stage=2
			elseif t-t_grip>t_timeout1 then
				motion_stage=2
			end
		end
	elseif motion_stage==2 then
		local angle_err=check_angle_err(cur_q,q_release)
		if t>t_release then
			if motion_type==1 or angle_err<5*DEG_TO_RAD or t-t_release>t_timeout2 then
				wcm.set_objects_selected(0)
				wcm.set_objects_enable(1)
				hcm.set_motionplan_execute(0)
				print("Approach time:",t-t_release)
				rospub.jointactiongoal(gripperjointnames, "test2",grip_fo, {0.5,1},1)
				motion_stage=3
				if motion_type==1 then t_end=t+2
				else t_end=t+1 end
			end
		end
	elseif motion_stage==3 and t>t_end then
		if hcm.get_motionplan_autorun()==0 then
			print("MOTION END")
			hcm.set_motionplan_execute(0)
			last_motion_execute=0
			motion_stage=0
		else
			--Wait for 3 seconds for image processing
			local objsel=wcm.get_objects_selected()
			if t>t_end+3.0 then
				hcm.set_motionplan_execute(0)
				last_motion_execute=0
				motion_stage=0
			elseif objsel>0 then
				hcm.set_motionplan_execute(1)
				last_motion_execute=0
				motion_stage=0
			end
		end
	end
end

local t_last_debug=0
local t0=unix.time()
local running=true
while running do
  update_jointstate()
	check_motionplan()
	motioncontrol()

  local t=unix.time()
  if t-t_last_debug>5 then
    print(string.format("ROSIO: %.1f sec Motion:%d",t-t0, motion_stage ))
    t_last_debug=t
  end
  unix.usleep(1E6*0.003)
end
