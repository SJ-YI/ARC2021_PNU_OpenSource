#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
if string.find(pwd,"Run") then  dofile('../include.lua')
else    dofile('../../include.lua') end


require'wcm'
local unix=require'unix'
local vector=require'vector'
local rossub=require'ur5_rossub'
local rospub=require'ur5_rospub'
local T = require'Transform'-----------------------------------------------------------------------

local t_last_update


local function entry()
  rossub.init('webots_ros')
  rospub.init('webots_ros')
  t_last_update=unix.time()
end


local function webots_tf()
  local pose=wcm.get_robot_pose()
  -- rospub.tf({pose[1],pose[2],0},{0,0,pose[3]}, "map","base_link")
  local qArm=Body.get_command_position()
  local pose=wcm.get_robot_pose()
  local qArm=Body.get_command_position()
  --Actual robot is mounted with 90 deg yaw angle offset
  --webots ur5 has shulder pitch angle offset (aiming +z)
  qArm[2]=qArm[2]- 90*DEG_TO_RAD
  rospub.jointstate(qArm)
end

local function webots_topcamera(seq, w,h, buf,fov)
  rospub.rgbimage2(seq,w,h,buf)
end

local function webots_rgbd(seq, w,h,whbuf, d_w, d_h, d_buf, fov)
   rospub.rgbimage(seq,w,h,whbuf)
   rospub.depthimage(seq,d_w,d_h,d_buf,1)
   rospub.camerainfo(seq,w,h,Config.kinect.fov)  
end

local function webots_lidar(seq, fov0,fov1,n,data,link)
  rospub.laserscan(seq,fov0,fov1,n,data,link)
end

local function webots_sub()
end


local function webots_item(name, obj_no,xyz, rpy)
  local objx=wcm.get_objects_xpos()
  local objy=wcm.get_objects_ypos()
  local objz=wcm.get_objects_zpos()
  local objnum=wcm.get_objects_num()
  local objids=wcm.get_objects_ids()
  local obj_id=0
  for i=1,#Config.item_list do
    if name==Config.item_list[i][1] then obs_id=i end
  end
  objx[obj_no]=xyz[1]
  objy[obj_no]=xyz[2]
  objz[obj_no]=xyz[3]
  objids[obj_no]=obs_id
  objnum=math.max(obj_no,objnum)
  wcm.set_objects_xpos(objx)
  wcm.set_objects_ypos(objy)
  wcm.set_objects_zpos(objz)
  wcm.set_objects_num(objnum)
  wcm.set_objects_ids(objids)
end


local function update()
  local t = rossub.rostime()
  wcm.set_robot_rostime(t)

  local t=unix.time()
  if t-t_last_update>1 then
    t_last_update=t
  end
end

if ... and type(...)=='string' then --webots handling
  WAS_REQUIRED = true
  return {
    entry=entry,
    update=update,
    exit=nil,
    webots_tf=webots_tf,
    webots_rgbd=webots_rgbd,
    webots_topcamera=webots_topcamera,
    webots_lidar=webots_lidar,
    webots_item=webots_item
    }
end

local running = true
local key_code
entry()
while running do
  update()
  unix.usleep(1E6*0.01)
end
