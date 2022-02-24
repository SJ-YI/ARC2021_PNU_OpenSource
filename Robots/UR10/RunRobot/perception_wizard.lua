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
require 'libObjects'

local rossub=require'rossub'
local rospub=require'rospub2'
local yolo2=require 'yolo2'



--FOR 2021 ARC challenge
--UR5e position: 0 0.7 0
--UR5e rotation: 1 0 0 4.71

--Camera position: 1.35 1.33 0
--Camera rotation: 0.357 -0.863 -0.357 -1.72
--Camera FOV: 0.7854 RAD
--CAMERA WF: 640 by 480

--Desk position: 0.578 -0.0122 0.0185
--Desk rotation: 0 1 0 -1.58

--LEFT TO RIGHT
--Paper crate: -0.4 0.19 0.8
--Can crate: -0.65 0.19 0.4
--Plastic crate: -0.65 0.19 0
--Glass crate: -0.65 0.19 -0.4
--Plasticbag crate: -0.4 0.19 -0.8

--Table max x: 0.9236
--Table max Y: 0.5737




local cameraparam={640,480,343.496368,343.496368,320.5, 240.5}
local xyz_limits={0.25,1.0,   -0.6, 0.6,   0.01,0.32} --xmin xmax ymin ymax zmin zmax
local p_camera={1.3527200222,0,0.6315,-2.3562, 0,1.571359418} --let's hard code it here


rossub.init('dockerio_ros')
--nodename image_topics lidar_topics path_topics occgrid_topics
rospub.init_custom('dockerio_ros',{"/depth_image_new","/pnu/rgb_rect_raw"},{},{"/path"},{"/lowres_map"})

local t_rgb, t_depth, t_yolo=0,0,0
local rgb_seq,d_seq=0,0
local t_img_sent
local rgb_camera_info, depth_camera_info=nil,nil
local rgbdata,depthdata,rgbdata2=nil,nil,nil
local rgb_frame=nil
local w_rgb,h_rgb,enc_rgb,w_d,h_d,enc_d
local yolo_sent=false



local sub_idx_YOLO=rossub.subscribeYOLO('/darknet_ros/bounding_boxes')
local sub_idx_hsrbcommand=rossub.subscribeInt32('/pnu/hsrbcommand')
local sub_idx_RGB=rossub.subscribeImage('/CAM/camera/image') --from ARC docker image
local sub_idx_D=rossub.subscribeImage('/CAM/range_finder/range_image') --from ARC docker image

local data= ffi.new("uint8_t[?]",1920*1080*3)

os.execute("mkdir -p ../Log/")
os.execute("touch ../Log/index.txt")

--flush all topics first
while rossub.checkYOLO(sub_idx_YOLO) do end
while rossub.checkImage(sub_idx_RGB) do end
while rossub.checkImage(sub_idx_D) do end

local t0=unix.time()
local t_last_update=unix.time()
local t_last_debug=unix.time()
local pose_count=0
local running = true
t_ros_last=rossub.rostime()
t_last_marker=unix.time()
local pose_old={0,0,0}



local yolo_interval_idle=Config.yolo_interval_idle
local yolo_interval_enable=Config.yolo_interval_enable
local rgb_sent_count=0
local yolo_detect_time=0

local temp=vector.zeros(255)
wcm.set_objects_xpos(temp)
wcm.set_objects_ypos(temp)
wcm.set_objects_zpos(temp)
wcm.set_objects_yaw(temp)
wcm.set_objects_id(temp)
wcm.set_objects_min_width(temp)
wcm.set_objects_max_width(temp)
wcm.set_objects_grabyaw(temp)



local function save_logs()
  print("LOGLOGLOG")
  local file=io.open("../Log/index.txt","r+")
  next_image_index=tonumber(file:read())
  file:close()
  if not next_image_index then next_image_index=0 end
  -- print(next_image_index)
  local file=io.open("../Log/index.txt","w")
  file:write(next_image_index+1)
  file:close()

  if Config.use_back_mask then
    print("try saving BLACKED RGB",#rgbdata, #rgbdata2)
    local logname=string.format("../Log/Log%04d.png",next_image_index)
    rospub.savergbimage(w_rgb,h_rgb,enc_rgb,rgbdata2,logname) --this seg faults
  else
    local logname=string.format("../Log/Log%04d.png",next_image_index)
    print("try saving ORGRGB")
    rospub.savergbimage(w_rgb,h_rgb,enc_rgb,rgbdata,logname)

  end


  print("try saving depth",#depthdata)
  local logname2=string.format("../Log/Depth/Log%04d.depth",next_image_index)
  local file2=io.open(logname2,"wb")
  file2:write(depthdata)
  file2:close()

  print("save done")
end




local function update_pose()
  -- local t_ros = rossub.rostime()
  -- if t_ros then
  --   wcm.set_robot_rostime(t_ros)
  --   -- print("rostime at perception:",t_ros)
  -- end
  local camera_pose=rossub.checkTF("world","head_rgbd_sensor_rgb_frame")
  if camera_pose then
    print("CAMERA POSE!",unpack(camera_pose))
    p_camera=camera_pose end
end




local function update_grip()
  print("===================================================================================")
  local objnum=wcm.get_objects_num()
  local oxp,oyp,ozp,oyaw,oid,ominw, omaxw=
    wcm.get_objects_xpos(),wcm.get_objects_ypos(),wcm.get_objects_zpos(),
    wcm.get_objects_yaw(),wcm.get_objects_id(),
    wcm.get_objects_min_width(),wcm.get_objects_max_width()
  local ograbyaw=wcm.get_objects_grabyaw()
  local yaws={180*DEG_TO_RAD, 135*DEG_TO_RAD, -135*DEG_TO_RAD, 90*DEG_TO_RAD, -90*DEG_TO_RAD, 0}



  local max_dist_wrist=0.9
  local wrist_length = 0.30
  local wrist_length = 0.40


  local min_dist_wrist=0.25
  local min_dist_wrist=0.40
  -- local wrist_x_check={-0.40,0.12}
  local wrist_x_check={-0.60,0.08} --for 8cm side move
  local dist_th = 0.10
  local dist_th = 0.09

  for i=1,objnum do
    local x,y=oxp[i],oyp[i]
    local good_yaw=nil
    for j=1,#yaws do
      if not good_yaw then
        local pose_grab={x,y,yaws[j]}
        local min_dist=999
        for ii=1,objnum do
          if ii~=i then
            local pose_abs={oxp[ii],oyp[ii],0}
            local rel_pose=util.pose_relative(pose_abs, pose_grab)
            if rel_pose[1]>wrist_x_check[1] and rel_pose[1]<wrist_x_check[2] then
              local y_dist=math.abs(rel_pose[2])-math.min(omaxw[ii]/2, 0.05)
              if y_dist<0.20 then
                if y_dist<min_dist then min_dist=y_dist end
              end
            end
          end
        end
        local pose_wrist=util.pose_global({-wrist_length,0,0},pose_grab)
        local dist_wrist=math.sqrt( pose_wrist[1]*pose_wrist[1]+pose_wrist[2]*pose_wrist[2] )
        if dist_wrist>=max_dist_wrist or dist_wrist<min_dist_wrist then
          print(string.format("Object %d (%.2f %.2f) yaw %d: wrist dist %.2f min_dist %.2f WRIST DIST ERR",
            i,x,y, yaws[j]/DEG_TO_RAD,dist_wrist, min_dist))
        elseif min_dist<dist_th then
          print(string.format("Object %d (%.2f %.2f) yaw %d: wrist dist %.2f min_dist %.2f MIN DIST ERR",
            i,x,y, yaws[j]/DEG_TO_RAD,dist_wrist, min_dist))
        else
          print(string.format("Object %d (%.2f %.2f) yaw %d: wrist dist %.2f min_dist %.2f",
            i,x,y, yaws[j]/DEG_TO_RAD,dist_wrist, min_dist))
        end



        if min_dist>dist_th and dist_wrist<max_dist_wrist and dist_wrist>min_dist_wrist then good_yaw=yaws[j] end
      end --end good_yaw
    end --end j

    if good_yaw then
      ograbyaw[i]=good_yaw
      print(string.format("====Object %d: good yaw %.1f====", i , good_yaw/DEG_TO_RAD))
    else
      ograbyaw[i]=-999
      print(string.format("====Object %d: no grab direction====", i))
    end
  end --end objnum
  wcm.set_objects_grabyaw(ograbyaw)
  print("===================================================================================")

  local best_item=0
  local best_dist=999
  for i=1,objnum do
    local camera_x=1.35
    local dist= (camera_x-oxp[i])*(camera_x-oxp[i]) + oyp[i]*oyp[i]
    if ograbyaw[i]>-999 and dist<best_dist then
      best_item, best_dist=i,dist
    end
  end

  if best_item>0 then
    local targettype=hcm.get_motionplan_targettype()
    targettype=targettype+1
    if targettype>5 then targettype=1 end

    if omaxw[best_item]>0.10 then
      hcm.set_motionplan_graspwidth(0.8) --semi grab
    else
      hcm.set_motionplan_graspwidth(1.1)  --full grab
    end

    hcm.set_motionplan_targettype(targettype)
    wcm.set_objects_selected(best_item)
    hcm.set_motionplan_targetxya({oxp[best_item], oyp[best_item], ograbyaw[best_item] })
    hcm.set_motionplan_targetsize(omaxw[best_item])

  else
    wcm.set_objects_selected(0)
  end
end



local function update_image()

  local w,h,enc,data,ts,frame=rossub.checkImage(sub_idx_RGB)
	if w then
		w_rgb,h_rgb,enc_rgb,rgbdata,tstamp=w,h,enc,data,ts
    rgb_frame=frame
		rgb_seq=rgb_seq+1
		t_rgb=unix.time()
    -- rospub.imageonly(rgb_seq,w_rgb,h_rgb,rgbdata,4,1,"head_rgbd_sensor_rgb_frame")--bgr8
    rospub.imageonly(rgb_seq,w_rgb,h_rgb,rgbdata,1,1,"head_rgbd_sensor_rgb_frame")--bgr8
	end

  local w,h,enc,data,ts,depth_frame=rossub.checkImage(sub_idx_D)
  if w then
    w_d,h_d,enc_d,depthdata,tstamp,depth_frame=w,h,enc,data,ts
    d_seq=d_seq+1
    t_depth=unix.time()
    rospub.camerainfo(d_seq,640,480,1.5)
    rospub.imageonly(d_seq,w_d,h_d,depthdata,2,0,"head_rgbd_sensor_rgb_frame") --32FC1

    if rgbdata and depthdata then
      if wcm.get_objects_enable()>0 then
      -- if d_seq%30==0 then
      -- if d_seq%5==0 then
        print("DEPTHCUT!!!")

        rgbdata2=yolo2.cleanrgb(cameraparam, depthdata,rgbdata, p_camera, xyz_limits)
        save_logs()

        local t0=unix.time()
        local numobj, xpos,ypos,zpos, yaw, minwidth, maxwidth=yolo2.depthcut(
          cameraparam,
          rgbdata,
          depthdata,
          p_camera,
          xyz_limits, --xmin xmax ymin ymax zmin zmax
          1 --debug
        )
        local t1=unix.time()

        if numobj>0 then
          wcm.set_objects_num(numobj)
          local oxp,oyp,ozp,oyaw,ominw, omaxw,oid=
            wcm.get_objects_xpos(),wcm.get_objects_ypos(),wcm.get_objects_zpos(),
            wcm.get_objects_yaw(),wcm.get_objects_min_width(),wcm.get_objects_max_width(),
            wcm.get_objects_id()

          for i=1,numobj do
            oxp[i],oyp[i],ozp[i],oyaw[i],oid[i]=xpos[i],ypos[i],zpos[i],yaw[i],i
            ominw[i],omaxw[i]=minwidth[i],maxwidth[i]

            print(string.format("Object %d: %.2f %.2f height %.2f yaw %.1f minwidth %.3f maxwidth %.3f",
            i,xpos[i],ypos[i],zpos[i], oyaw[i]/DEG_TO_RAD, minwidth[i],maxwidth[i]))

          end
          wcm.set_objects_xpos(oxp)
          wcm.set_objects_ypos(oyp)
          wcm.set_objects_zpos(ozp)
          wcm.set_objects_yaw(oyaw)
          wcm.set_objects_id(oid)
          wcm.set_objects_min_width(ominw)
          wcm.set_objects_max_width(omaxw)
        else
          wcm.set_objects_num(0)
        end
        update_grip()
        wcm.set_objects_enable(0)
        print("Time elapsed:",(t1-t0)*1000,"ms")
      end
    end
  end --end w
end



local function update_hsrbcmd()
  local ret=rossub.checkInt32(sub_idx_hsrbcommand)
  if ret then
    print(ret)
    if ret==4 and rgbdata then --image log
      print("LOGLOGLOG")
      local file=io.open("../Log/index.txt","r+")
      next_image_index=tonumber(file:read())
      file:close()
      if not next_image_index then next_image_index=0 end
      -- print(next_image_index)
      local file=io.open("../Log/index.txt","w")
      file:write(next_image_index+1)
      file:close()
      local logname=string.format("../Log/Log%04d.png",next_image_index)
      print("try saving")
      rospub.savergbimage(w_rgb,h_rgb,enc_rgb,rgbdata,logname)
      local logname2=string.format("../Log/Log%04d.depth",next_image_index)
      local file2=io.open(logname2,"wb")
      file2:write(depthdata)
      file2:close()
      print("save done")
    end
  end
end






local function update_markers()
  local objnum=wcm.get_objects_num()
  local oxp,oyp,ozp,oyaw,oid,ominw, omaxw=
    wcm.get_objects_xpos(),wcm.get_objects_ypos(),wcm.get_objects_zpos(),
    wcm.get_objects_yaw(),wcm.get_objects_id(),
    wcm.get_objects_min_width(),wcm.get_objects_max_width()
  local ograbyaw=wcm.get_objects_grabyaw()

  local sel=wcm.get_objects_selected()
  if objnum>0 then
    mc=0
    local posx,posy,posz,orir,orip,oriy={},{},{},{},{},{}
    local scalex,scaley,scalez={},{},{}
    local names,colors,alphas,types={},{},{},{}
    local mc=0
    for i=1,objnum do
      local color=1
      if i==sel then color=2 end
      mc=mc+1
      types[mc],posx[mc],posy[mc],posz[mc],orir[mc],orip[mc],oriy[mc],scalex[mc],scaley[mc],scalez[mc], names[mc],colors[mc],alphas[mc]=
       5, oxp[i],oyp[i],ozp[i]/2,0,0,oyaw[i], ominw[i], omaxw[i], ozp[i],  "x", color, 1

      if ograbyaw[i]>-999 then
       local grabd=0.10
        mc=mc+1
        types[mc],posx[mc],posy[mc],
        posz[mc],orir[mc],orip[mc],oriy[mc],scalex[mc],scaley[mc],scalez[mc], names[mc],colors[mc],alphas[mc]=
         5,
         oxp[i]-grabd*math.cos(ograbyaw[i])/2,
         oyp[i]-grabd*math.sin(ograbyaw[i])/2,
         0.05 ,0,0,ograbyaw[i], grabd, 0.01, 0.10,  "x", 4, 1
      end

    end
    rospub.marker3d(mc,types,   posx,posy,posz,  orir,orip,oriy,  scalex,scaley,scalez,names,colors,alphas)
  end
end

while running do

  -- rospub.tf({0,0,0},{0,0,0}, "map","world")
  -- rospub.tf({1.35272,0,0.6315},{0,-2.3562,0}, "world","camera0")
  -- rospub.tf({0,0,0},{0,0,1.57}, "camera0","head_rgbd_sensor_rgb_frame")
  -- rospub.tf({0,0,0},{0,0,1.57}, "camera0","CAM/camera") --this is provided by webots?
  -- rospub.tf({0,0,0},{0,0,0}, "head_rgbd_sensor_rgb_frame","CAM/camera")

  -- update_pose()
  update_image()
  update_hsrbcmd()
  -- update_yolo()

  local t=unix.time()
  if t-t_last_marker>0.1 then
    update_markers()
    t_last_marker=t
  end
  if t-t_last_debug>5 then
    local pose=wcm.get_robot_pose()
    -- print(string.format("Perception wizard%.3f Hz",pose_count/(t-t_last_debug)))
    t_last_debug=t
    pose_count=0
  end

  unix.usleep(1E6*0.003)
end
