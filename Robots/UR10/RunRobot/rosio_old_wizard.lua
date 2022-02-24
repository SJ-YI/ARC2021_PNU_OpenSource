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
-- K.setup_tool_param(0.16,0)


local sub_idx_jointstate=rossub.subscribeJointState('/joint_states')
local sub_idx_gripperjointstate=rossub.subscribeJointState('/gripper_joint_states')

local jointnames={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"}
local gripperjointnames={"finger_1_joint_1","finger_2_joint_1","finger_middle_joint_1"}

local cur_q={0,0,0,0,0,0}
local grip_fo=vector.ones(6)*0.1
local grip0=vector.ones(6)*0.1 --full open
local grip0=vector.ones(6)*0.2 --full open
local grip1=vector.ones(6)*0.8 --semi close

local move_timing1=1
local pickup_timing={0.5,0.5, 1, 1}--down side hold up

ang_vel=90*DEG_TO_RAD

local move_timing3=0.5
local release_timing=1

K.setup_tool_param(0.13,0)
local side_move,down_move=0.13,0.40
local side_move,down_move=0.08,0.40
local z_height0=0.07

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
	local j1_old,j5_old=j0[1],j0[5]
	for i=0,div do
		local ph=i/div
		local p1={p0[1]+ph*xyz[1],p0[2]+ph*xyz[2],p0[3]+ph*xyz[3],p0[4],p0[5],p0[6]}
		-- q1=vector.new(K.inverse_arm(p1, j0,2))
		-- q1=vector.new(K.inverse_arm(p1, j0,4))
		q1=vector.new(K.inverse_arm(p1, j0,5))

--[[
		local p2=K.forward_arm(q1)
		q2=q1/DEG_TO_RAD
		for kk=1,6 do if q1[kk]~=q1[kk] then print("NAN NAN NAN!!!!") end end
		print(string.format("p0: (%.3f %.3f %.3f) p1: (%.3f %.3f %.3f) q: %.1f %.1f %.1f %.1f %.1f %.1f",
		  p1[1],p1[2],p1[3],
			p2[1],p2[2],p2[3],
			q2[1],q2[2],q2[3],q2[4],q2[5],q2[6]
		))
--]]

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



local last_motion_execute=0

local jtraj_all, dur_all
local t_motion_start,t_grip, t_release,t_end
local q_grip, q_release

local motion_stage=0
hcm.set_motionplan_execute(0)
local motion_type=0

local function update_motionplan()
	motion_type=0
	local targettype=hcm.get_motionplan_targettype()
	local targetsize=hcm.get_motionplan_targetsize()
	local targetxya=hcm.get_motionplan_targetxya()
	local targetid=hcm.get_motionplan_targetid()

	local sel=wcm.get_objects_selected()


	local side_move,down_move=0.08,0.40
	-- local side_move,down_move=0.0,0.40


	print(string.format("MOTION EXECUTE, Target:(%.4f %.4f %.1f) id:%d size:%.3f type:%d",
		targetxya[1],targetxya[2],targetxya[3]/DEG_TO_RAD,
		targetid, targetsize, targettype ))

	local tool_param_delta,grip_target,gripopen_target=0,hcm.get_motionplan_graspwidth(),0.2
	if targetsize>0.10 then targetsize=0.10 end
		print("Item radius::::",targetsize/2)
	if targetsize/2>0.04 then
		tool_param_delta=-0.01
		gripopen_target=0.2 --full open
		grip_target=0.7
	else
		-- gripopen_target=0.4 --little open
		gripopen_target=0.2 --little open
		grip_target=1.1
	end


--coffee: 3
--small milk:4
--wind:5
--watter:6



	--WINE handling
	if targetid==5 then
		print("WINE WINE WINE!")
		tool_param_delta,grip_target,gripopen_target=-0.03,0.7,0.1

	elseif targetid==4 then --small milk
		tool_param_delta,grip_target=-0.02,1.1
	end

	K.setup_tool_param(0.13+tool_param_delta,0)--push closer
	grip0=vector.ones(6)*gripopen_target
	grip1=vector.ones(6)*grip_target


	local jangle0=vector.new({0, -45, 60, -45,0,0})*DEG_TO_RAD
	local jangle1=vector.new({0, 70, 20, -0,-0,0})*DEG_TO_RAD --facing right (from robot)

	local p1={targetxya[1]-side_move*math.cos(targetxya[3]),
						targetxya[2]-side_move*math.sin(targetxya[3]),
					z_height0+down_move,3.141592, 0*DEG_TO_RAD, targetxya[3] } --gripper facing backward (from robot)

	-- local q1=vector.new(K.inverse_arm(p1, jangle1,2))
	-- local q1=vector.new(K.inverse_arm(p1, jangle1,4))
	local q1=vector.new(K.inverse_arm(p1, jangle1,5))

	local p_can={-0.4,-0.8,0.10,180*DEG_TO_RAD,90*DEG_TO_RAD,180*DEG_TO_RAD} --plastic

	if targettype==1 then p_can[1],p_can[2]=-0.65, 0 -- plastic
	elseif targettype==2 then p_can[1],p_can[2]=-0.4, -0.8 -- paper
	elseif targettype==3 then p_can[1],p_can[2]=-0.65, -0.4 --can
	elseif targettype==4 then p_can[1],p_can[2]=-0.4, 0.8 --vinyl
	elseif targettype==5 then p_can[1],p_can[2]=-0.65, 0.4 --bottle
	end

	-- local q_can=vector.new(K.inverse_arm(p_can, jangle1,4))
	local q_can=vector.new(K.inverse_arm(p_can, jangle1,5))
	q_can[6]=0 --zero wrist spin

	-- if q_can[1]<0 then q_can[1]=q_can[1]+360*DEG_TO_RAD end --only rotate left (prevent item hitting)



	local jtraj1, dur1=jangle_interpolate(jangle0, q1, move_timing1, 100)

	local jtraj2, dur2,q2_fin=IK_interpolate(q1, {0,0,-down_move}, pickup_timing[1], 100)
	local jtraj22,dur22,q2_fin=IK_interpolate(q2_fin, {side_move*math.cos(targetxya[3]),side_move*math.sin(targetxya[3]),0}, pickup_timing[2], 100)
	q_grip=util.shallow_copy(q2_fin)

	local jtrajstop, durstop=jangle_interpolate(q2_fin, q2_fin, pickup_timing[3], 100)
	local jtraj3, dur3,q3_fin=IK_interpolate(q2_fin, {0,0,down_move}, pickup_timing[4], 100) --raise up

	local q_can_high=vector.new({q_can[1],q3_fin[2],q3_fin[3],q3_fin[4],q3_fin[5],q3_fin[6]}) --target location

	move_timing2=math.abs(q3_fin[1]-q_can_high[1])/ang_vel
	print("rotate time:",move_timing2)

	--PREVENT overturning of the 5th joint
	if q_can[5]>q_can_high[5]+180*DEG_TO_RAD then q_can[5]=q_can[5]-360*DEG_TO_RAD end
	if q_can[5]<q_can_high[5]-180*DEG_TO_RAD then q_can[5]=q_can[5]+360*DEG_TO_RAD end

	local jtraj4, dur4=jangle_interpolate(q3_fin, q_can_high, move_timing2, 100)
	local jtraj5, dur5=jangle_interpolate(q_can_high, q_can, move_timing3, 100)


	q_release=util.shallow_copy(q_can)

	jtraj_all,dur_all=traj_concat(jtraj1, jtraj2, dur1, dur2)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj22, dur_all, dur22)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtrajstop, dur_all, durstop)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj3, dur_all, dur3)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj4, dur_all, dur4)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj5, dur_all, dur5)

	local t=unix.time()
	t_grip= t + move_timing1 + pickup_timing[1]+pickup_timing[2]
	t_release= t_grip + pickup_timing[3]+pickup_timing[4]+move_timing2+move_timing3
	t_end = t_release+ release_timing
	motion_stage=1

	-- print("Grip jangle:", unpack(vector.new(q_grip)/DEG_TO_RAD))
	-- print("Release jangle:", unpack(vector.new(q_release)/DEG_TO_RAD))
	rospub.jointactiongoal(jointnames, "test1",jtraj_all, dur_all,0)
	rospub.jointactiongoal(gripperjointnames, "test2",grip0, {0.5,1},1)
end


local function update_motionplan2() --for vertical pickup
	motion_type=1
	local targettype=hcm.get_motionplan_targettype()
	local targetsize=hcm.get_motionplan_targetsize()
	local targetxya=hcm.get_motionplan_targetxya()


	local objsel=wcm.get_objects_selected()
	local ominw=wcm.get_objects_min_width()
	local omaxw=wcm.get_objects_max_width()

	-- local objyaw=wcm.get_objects_yaw()
	-- targetxya[3]=objyaw[objsel]+90*DEG_TO_RAD
	-- if targetxya[3]>180*DEG_TO_RAD then targetxya[3]=targetxya[3]-360*DEG_TO_RAD end

	print(string.format("MOTION EXECUTE, Target:%.3f %.3f (%.1f)",targetxya[1],targetxya[2],targetxya[3]/DEG_TO_RAD))
	grip1=vector.ones(6)*0.8 --for snack
	grip1=vector.ones(6)*0.9 --for snack
	K.setup_tool_param(0.13,0)--default, for small items


	local z_height_vg=0.07
	local jangle0=vector.new({0, -45, 60, -45,0,0})*DEG_TO_RAD
	local jangle1=vector.new({0, 70, 20, -0,-0,0})*DEG_TO_RAD --facking right (from robot)

	-- local p1={targetxya[1],targetxya[2],z_height_vg+down_move,3.141592, 90*DEG_TO_RAD, targetxya[3] } --gripper facing backward (from robot)

	local side_offset=0

	local p1={targetxya[1]-side_offset*math.sin(targetxya[3]),
						targetxya[2]-side_offset*math.cos(targetxya[3]),
					z_height_vg+down_move,3.141592, 90*DEG_TO_RAD, targetxya[3] } --gripper facing backward (from robot)
	-- local q1=vector.new(K.inverse_arm(p1, jangle1,4))
	local q1=vector.new(K.inverse_arm(p1, jangle1,5))
	local p_can={-0.4,-0.8,0.20,180*DEG_TO_RAD,90*DEG_TO_RAD,180*DEG_TO_RAD}

	if targettype==1 then p_can[1],p_can[2]=-0.65, 0 -- plastic
	elseif targettype==2 then p_can[1],p_can[2]=-0.4, -0.8 -- paper
	elseif targettype==3 then p_can[1],p_can[2]=-0.65, -0.4 --can
	elseif targettype==4 then p_can[1],p_can[2]=-0.4, 0.8 --vinyl
	elseif targettype==5 then p_can[1],p_can[2]=-0.65, 0.4 --bottle
	end

	-- local q_can=vector.new(K.inverse_arm(p_can, jangle1,4))
	local q_can=vector.new(K.inverse_arm(p_can, jangle1,5))
	-- q_can[6]=0 --zero wrist spin

	if q_can[6]>90*DEG_TO_RAD then q_can[6]=q_can[6]-180*DEG_TO_RAD end
	if q_can[6]<-90*DEG_TO_RAD then q_can[6]=q_can[6]+180*DEG_TO_RAD end


	local jtraj1, dur1=jangle_interpolate(jangle0, q1, move_timing1, 100)
	local jtraj2, dur2,q2_fin=IK_interpolate(q1, {0,0,-down_move}, pickup_timing[1], 100)

	q_grip=util.shallow_copy(q2_fin)

	local jtrajstop, durstop=jangle_interpolate(q2_fin, q2_fin, pickup_timing[3], 100)
	local jtraj3, dur3,q3_fin=IK_interpolate(q2_fin, {0,0,down_move}, pickup_timing[4], 100) --raise up

	local q_can_high=vector.new({q_can[1],q3_fin[2],q3_fin[3],q3_fin[4],q3_fin[5],q3_fin[6]}) --target location

	move_timing2=math.abs(q3_fin[1]-q_can_high[1])/ang_vel
	print("rotate time:",move_timing2)

	--PREVENT overturning of the 5th joint
	if q_can[5]>q_can_high[5]+180*DEG_TO_RAD then q_can[5]=q_can[5]-360*DEG_TO_RAD end
	if q_can[5]<q_can_high[5]-180*DEG_TO_RAD then q_can[5]=q_can[5]+360*DEG_TO_RAD end
	local jtraj4, dur4=jangle_interpolate(q3_fin, q_can_high, move_timing2, 100)
	local jtraj5, dur5=jangle_interpolate(q_can_high, q_can, move_timing3, 100)


	local downpush={0.10,0,-0.40}
	local uppush={-0.10,0,0.40}

	local jtraj6, dur6,q5_fin=IK_interpolate(q_can, downpush, 1, 100) --raise up
	local jtraj7, dur7,q6_fin=IK_interpolate(q5_fin, uppush, 1, 100) --raise up


	q_release=util.shallow_copy(q_can)

	jtraj_all,dur_all=traj_concat(jtraj1, jtraj2, dur1, dur2)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtrajstop, dur_all, durstop)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj3, dur_all, dur3)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj4, dur_all, dur4)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj5, dur_all, dur5)

	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj6, dur_all, dur6)
	jtraj_all,dur_all=traj_concat(jtraj_all, jtraj7, dur_all, dur7)

	local t=unix.time()
	t_grip= t + move_timing1 + pickup_timing[1]+pickup_timing[2]
	t_release= t_grip + pickup_timing[3]+pickup_timing[4]+move_timing2+move_timing3
	t_end = t_release+ release_timing + 3
	motion_stage=1

	-- print("Grip jangle:", unpack(vector.new(q_grip)/DEG_TO_RAD))
	-- print("Release jangle:", unpack(vector.new(q_release)/DEG_TO_RAD))

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
