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
local xyz_limits={0.24,1.0,   -0.6, 0.6,   0.01,0.32} --xmin xmax ymin ymax zmin zmax
local p_camera={1.3527200222,0,0.6315,-2.3562, 0,1.571359418} --let's hard code it here


rossub.init('dockerio_ros')
rospub.init_custom('dockerio_ros',{"/depth_image_new","/rgb_image_new","/mask_rcnn/input"},{},{"/path"},{"/lowres_map"}) --nodename image_topics lidar_topics path_topics occgrid_topics

local t_rgb, t_depth, t_yolo=0,0,0
local rgb_seq,d_seq=0,0
local t_img_sent
local rgb_camera_info, depth_camera_info=nil,nil
local rgbdata,rgbdata2,depthdata=nil,nil,nil
local maskdata=nil

local mask_image_data=nil
local rgb_frame=nil
local w_rgb,h_rgb,enc_rgb,w_d,h_d,enc_d
local mrcnn_sent=false


local sub_idx_MRCNNimage=rossub.subscribeImage('/mask_rcnn/result')
local sub_idx_MRCNN=rossub.subscribeImage('/mask_rcnn/result_mask')
local sub_idx_hsrbcommand=rossub.subscribeInt32('/pnu/hsrbcommand')
local sub_idx_RGB=rossub.subscribeImage('/CAM/camera/image') --from ARC docker image
local sub_idx_D=rossub.subscribeImage('/CAM/range_finder/range_image') --from ARC docker image

-- local data= ffi.new("uint8_t[?]",1920*1080*3)

os.execute("mkdir -p ../Log/")
os.execute("mkdir -p ../Log/ARC")
os.execute("mkdir -p ../Log/ARC/MASK")
os.execute("mkdir -p ../Log/ARC/Depth")
os.execute("mkdir -p ../Log/ARC/RGB")
os.execute("touch ../Log/index.txt")

--flush all topics first

while rossub.checkImage(sub_idx_RGB) do end
while rossub.checkImage(sub_idx_D) do end
while rossub.checkImage(sub_idx_MRCNNimage) do end
while rossub.checkImage(sub_idx_MRCNN) do end


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

local yolo_interval=1
local yolo_start_delay=1
local rgb_ready,d_ready=false,false
local mrcnn_timeout=3


local typenames={"Plastic","Paper","Can","Vinyl","Bottle"}

local classnames={
  {"coke", "Coke Can",3},
  {"beer", "Beer Can",3},
  {"coffee", "Coffee",2}, --coffee is now paper
  {"milk","Small Milk",2},
  {"wine","Wine",5},
  {"water","Water Bottle",1},
  {"coke_big","Coke Bottle",1},--NOT a 2 liter bottle, though
  {"milk_big","Big Milk",2},
  {"beer_big","Beer Bottle",5},
  {"snack","Snack",4},
  {"paper","Toilet Paper",2},
  {"painkiller","Painkiller",1},
}


--Class IDs defined in mrcnn json file (DIFFERENT FROM DEF HERE!!!)
--2021/1013 weight
-- local classlist={"snack","painkiller","milk","coffee","milk_big","beer_big","wine","beer","coke","coke_big","water","paper"}

--2021/10/23 weight
local classlist={"painkiller","milk","coke","milk_big","coke_big","beer_big",
                  "wine","paper","coffee","water","beer","snack"}



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

  print("SAVING FILE!!!")
  if rgbdata then
    local logname=string.format("../Log/ARC/RGB/Log%04d.png",next_image_index)
    print(#rgbdata, w_rgb*h_rgb*4, enc_rgb)
    rospub.savergbimage(w_rgb,h_rgb,enc_rgb,rgbdata,logname)
  else
    print("NO RGBDATA")
  end


  if mask_image_data then
    local logname=string.format("../Log/ARC/MASK/Log%04dMASK.png",next_image_index)
    print("try saving MASKRCNN results")
    rospub.savergbimage(w_rgb,h_rgb,"bgr8",mask_image_data,logname)
  end

  -- print("try saving depth",#depthdata)
  -- local logname2=string.format("../Log/ARC/Depth/Log%04d.depth",next_image_index)
  -- local file2=io.open(logname2,"wb")
  -- file2:write(depthdata)
  -- file2:close()

  print("save done")
end


local function update_image()
  local w,h,enc,data,ts,frame=rossub.checkImage(sub_idx_RGB)
	if w then
		w_rgb,h_rgb,enc_rgb,rgbdata,tstamp=w,h,enc,data,ts --BGRA8
    rgb_frame=frame
		rgb_seq=rgb_seq+1
		t_rgb=unix.time()
    -- print("RGB",enc_rgb,w_rgb,h_rgb)
    -- rospub.imageonly(rgb_seq,w_rgb,h_rgb,rgbdata,1,1,"head_rgbd_sensor_rgb_frame") --BGRA8
    rospub.imageonly(rgb_seq,w_rgb,h_rgb,rgbdata,5,1,"head_rgbd_sensor_rgb_frame") --BGRA8 to BGR8
    rgb_ready=true
	end

  local w,h,enc,data,ts,depth_frame=rossub.checkImage(sub_idx_D)
  if w then
    w_d,h_d,enc_d,depthdata,tstamp,depth_frame=w,h,enc,data,ts
    d_seq=d_seq+1
    t_depth=unix.time()
    -- print("D",enc_rgb,w_rgb,h_rgb)
    rospub.camerainfo(d_seq,640,480,1.5)
    rospub.imageonly(d_seq,w_d,h_d,depthdata,2,0,"head_rgbd_sensor_rgb_frame") --32FC1
    d_ready=true
  end

  local t=unix.time()
  if rgb_ready and d_ready and wcm.get_objects_enable()>0 then
    print("MRCNN SENT!!!")

    while rossub.checkImage(sub_idx_MRCNNimage) do end
    while rossub.checkImage(sub_idx_MRCNN) do end

    wcm.set_objects_num(0)
    rospub.tf({0,0,0},{0,0,0}, "map","world")
    rospub.tf({1.35272,0,0.6315},{0,-2.3562,0}, "world","camera0")
    rospub.tf({0,0,0},{0,0,1.57}, "camera0","CAM/camera")
    rospub.tf({0,0,0},{0,0,1.57}, "camera0","head_rgbd_sensor_rgb_frame")

    --Send rgb image to mrcnn process
    rospub.imageonly(rgb_seq,w_rgb,h_rgb,rgbdata,5,2,"head_rgbd_sensor_rgb_frame") --BGRA8 to BGR8
    -- print(string.format("MRCNN INPUT IMAGE: w%d h%d",w_rgb,h_rgb))
    unix.usleep(1E6*0.03)
    rgb_seq=rgb_seq+1

    t_yolo=unix.time()
    mrcnn_sent=true
    print("MRCNN SENT!!==================================")
    rgb_ready,d_ready=false,false
  end
end


local table_height=0.01
local show_debug=0

local function find_item_id(name)
  for i=1,#classnames do
    if name==classnames[i][1] then return i end
  end
  return 0
end

local function update_mrcnn()
  local temp=true
  local delay1, delay2, delay3=0,0,0
  if mrcnn_sent then
    local loop_start=unix.time()

    while true do
      --Dump all the incoming rgb and depth data
      while rossub.checkImage(sub_idx_RGB) do end
      while rossub.checkImage(sub_idx_D) do end
      local w,h,enc,data,ts,frame=rossub.checkImage(sub_idx_MRCNN)
      if w then
        maskdata=data
        delay1=unix.time()-t_yolo
        -- print(string.format("Mask image : w%d h%d enc%s %d",w,h,enc,#data))
        local t0=unix.time()
        mask_image_data=nil
        local yolo_img_loop=true
        local w,h,enc,yolodata,ts,depth_frame=rossub.checkImage(sub_idx_MRCNNimage)
        if yolodata then yolo_img_loop,mask_image_data=false,yolodata end
        while yolo_img_loop do
          w,h,enc,yolodata,ts,depth_frame=rossub.checkImage(sub_idx_MRCNNimage)
          if yolodata then yolo_img_loop,mask_image_data=false,yolodata end
          local t=unix.time()
          if t-t0>1.0 then print("MRCNN IMAGE RECEIVE TIMEOUT!!!") end
        end
        delay2=unix.time()-t_yolo


        local min_check_z=0.03
        local det_num, det_names, det_xpos, det_ypos, det_zpos, det_yaw, det_width, det_minwidth, rgbdata2 =yolo2.maskdetect(
        cameraparam,rgbdata,depthdata,p_camera,xyz_limits,maskdata,classlist, min_check_z,1)

        print("MRCNN: 3D objects "..det_num)
        local oxp,oyp,ozp,    oyaw,ominw,omaxw,  oid=
         wcm.get_objects_xpos(),wcm.get_objects_ypos(),wcm.get_objects_zpos(),
         wcm.get_objects_yaw(),wcm.get_objects_min_width(),wcm.get_objects_max_width(),
         wcm.get_objects_id()

        local item_count=0
        for i=1,det_num do
         local object_id=find_item_id(det_names[i])
         local add_item=true

         --MILK detected at the robot check
         if det_xpos[i]<0.30 and math.abs(det_ypos[i]+0.113)<0.10 then
           if object_id==8 then   print("FANTHOM MILK!!");   add_item=false end
           if object_id==11 then   print("FANTHOM PAPER!!");   add_item=false end
         end

         -- if det_xpos[i]~=det_xpos[i] or det_ypos[i]~=det_ypos[i] then add_item=false end

         --due to mask shrink, sometimes we only see the frontal pixels




         local aratio=det_width[i]/det_minwidth[i]
         local z_comp_factor=0.7
         local aratio_min=1.3
         if aratio>aratio_min and object_id~=10 and det_zpos[i]>0.06 and det_width[i]<0.10 then
           det_xpos[i]=det_xpos[i]+det_minwidth[i]-det_width[i]*z_comp_factor
           det_minwidth[i]=det_width[i]
         end

         if det_names[i]=="coke" then
           print(string.format("width: %.2f %.2f aratio:%.1f ",det_minwidth[i],det_width[i],aratio)  )


         end

         if add_item then
           item_count=item_count+1
           oxp[item_count],oyp[item_count],ozp[item_count],oyaw[item_count]=det_xpos[i],det_ypos[i],det_zpos[i],det_yaw[i]
           ominw[item_count],omaxw[item_count]=det_minwidth[i],det_width[i]
           oid[item_count]=find_item_id(det_names[i])
         end
        end

        wcm.set_objects_num(item_count)
        wcm.set_objects_xpos(oxp)
        wcm.set_objects_ypos(oyp)
        wcm.set_objects_zpos(ozp)
        wcm.set_objects_yaw(oyaw)
        wcm.set_objects_id(oid)
        wcm.set_objects_min_width(ominw)
        wcm.set_objects_max_width(omaxw)
        if item_count>0  then hcm.set_motionplan_gripexecute(1) end
        delay3=unix.time()-t_yolo
        print(string.format("MRCNN detection : mrcnn %.2f sec image %.2f sec detect %.2f sec",delay1, delay2-delay1, delay3-delay2))
        break
      end --end w
      if unix.time()>loop_start+mrcnn_timeout then print("MRCNN Timeout!!!!"); break end
      unix.usleep(1E6*0.005)
    end --end while true
    wcm.set_objects_enable(0)
  end --end mrcnn_sent
  mrcnn_sent=false
  t_last_marker= 0 --force update marker
end

local function update_hsrbcmd()
  local ret=rossub.checkInt32(sub_idx_hsrbcommand)
  if ret then
    print(ret)
    if ret==4 and rgbdata then --image log
      save_logs()
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
  local ograbtype=wcm.get_objects_grabtype() --0 for side, 1 for direct vertical, 2 for offset vertical (bag)

  if objnum>0 then
    local mc=0
    local posx,posy,posz,yaw,orir,orip,oriy={},{},{},{},{},{},{}
    local scales,scalex,scaley,scalez={},{},{},{}
    local names,colors,alphas,types={},{},{},{}

    for i=1,objnum do
      -- local color,scale=6,0.5
      local color,scale=6,0.5
      if i==sel then color=5,0.75 end
      mc=mc+1 --object name marker
      local objfullname="unknown"
      if oid[i]>0 then  objfullname=classnames[oid[i]][1] end
      types[mc],posx[mc],posy[mc],posz[mc],yaw[mc], names[mc], scales[mc],colors[mc]=
        2, oxp[i],oyp[i],ozp[i]+0.03,0,objfullname, scale, color
      -- local scale,color=1,1
      -- mc=mc+1 --cylinder marker
      -- types[mc],posx[mc],posy[mc],posz[mc],yaw[mc],names[mc], scales[mc],colors[mc]= 1, oxp[i], oyp[i],ozp[i]/2,yaw[i],"x" , scale, color

      -- if ograbyaw[i]>-999 then --ARROW
      --
      --   local scale,color=0.4,1
      --   if i==sel then color=4 end
      --   mc=mc+1
      --   if ograbtype[i]==0 then
      --     types[mc],posx[mc],posy[mc],posz[mc],yaw[mc], names[mc], scales[mc],colors[mc]=
      --       5, oxp[i]-math.cos(ograbyaw[i])*scale, oyp[i]-math.sin(ograbyaw[i])*scale,
      --         ozp[i]/2,ograbyaw[i],"x", scale, color
      --   else --vertical
      --
      --     types[mc],posx[mc],posy[mc],posz[mc],yaw[mc], names[mc], scales[mc],colors[mc]=
      --       1, oxp[i], oyp[i],ozp[i]+(scale*2)*0.10,ograbyaw[i],"x", scale*4, color
      --   end
      -- end


    end
    rospub.marker(mc,types,posx,posy,posz,yaw,names,scales,colors)
    posx,posy,posz,orir,orip,oriy={},{},{},{},{},{}
    names,colors,alphas,types={},{},{},{}

    mc=0
    for i=1,objnum do
      local color=2
      -- if i==sel then color=4 end
      mc=mc+1

      --THIS DRAWS THE OUTLINE
      -- types[mc],posx[mc],posy[mc],posz[mc],orir[mc],orip[mc],oriy[mc],scalex[mc],scaley[mc],scalez[mc], names[mc],colors[mc],alphas[mc]=
      --    5, oxp[i],oyp[i],ozp[i]/2,      0,0,oyaw[i],    ominw[i], omaxw[i], ozp[i],  "x", color, 1

      local markertype=3
      if ograbtype[i]~=0 then markertype=1 end
      types[mc],posx[mc],posy[mc],posz[mc],orir[mc],orip[mc],oriy[mc],scalex[mc],scaley[mc],scalez[mc], names[mc],colors[mc],alphas[mc]=
         markertype, oxp[i],oyp[i],ozp[i]/2,      0,0,oyaw[i],    ominw[i], omaxw[i], ozp[i],  "x", color, 0.5

      if hcm.get_motionplan_show_arrow()>0 then
        if ograbyaw[i]>-999 then --ARROW
          local arrowlen,arrowwid,color,alpha=0.4,0.05,6,0.5
          if i==sel then arrowwid,color,alpha=    0.06,5,0.8 end
          mc=mc+1
          if ograbtype[i]==0 then
            types[mc],posx[mc],posy[mc],posz[mc],orir[mc],orip[mc],oriy[mc],scalex[mc],scaley[mc],scalez[mc], names[mc],colors[mc],alphas[mc]=
                 0, oxp[i]-math.cos(ograbyaw[i])*arrowlen, oyp[i]-math.sin(ograbyaw[i])*arrowlen, 0.10, 0,0,ograbyaw[i],   arrowlen, arrowwid, arrowwid,  "x", color, alpha
          else --vertical
            types[mc],posx[mc],posy[mc],posz[mc],orir[mc],orip[mc],oriy[mc],scalex[mc],scaley[mc],scalez[mc], names[mc],colors[mc],alphas[mc]=
                 0, oxp[i], oyp[i], arrowlen, 0,90*DEG_TO_RAD,0,   arrowlen, arrowwid, arrowwid,  "x", color, alpha
          end
        end
      end
    end
    rospub.marker3d(mc,types,   posx,posy,posz,  orir,orip,oriy,  scalex,scaley,scalez,names,colors,alphas)
  end
end


while running do
  update_image()
  update_hsrbcmd()
  update_mrcnn()

  local t=unix.time()
  if t-t_last_marker>0.2 then
    update_markers()
    t_last_marker=t
  end
  if t-t_last_debug>5 then
    local pose=wcm.get_robot_pose()
    print(string.format("Perception MRCNN %.3f Hz",pose_count/(t-t_last_debug)))
    t_last_debug=t
    pose_count=0
  end

  unix.usleep(1E6*0.003)
end
