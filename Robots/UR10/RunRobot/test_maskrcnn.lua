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
local rossub=require'rossub'
local yolo2=require 'yolo2'

-- rospub.init_custom('mask_test',{"/CAM/range_finder/range_image","/CAM/camera/image","/darknet_ros/detection_image"},{},{"/path"},{"/lowres_map"})
-- rospub.init_custom('mask_test',{"/depth_image_new","/CAM/camera/image","/mask_image"},{},{"/path"},{"/lowres_map"})
rospub.init_custom('mask_test',{"/depth_image_new","/CAM/camera/image","/darknet_ros/detection_image","maskimage"},{},{"/path"},{"/lowres_map"})
rossub.init('mark_test')


local filename="../Log/Log3526"
-- local filename="../Log/Log3527"
-- local filename="../Log/Log3528"
-- local filename="../Log/Log3529"
-- local filename="../Log/Log3530"
-- local filename="../Log/Log3531"
-- local filename="../Log/Log3532"
-- local filename="../Log/Log3533" --ERROR
-- local filename="../Log/Log3534"
-- local filename="../Log/Log3535"




local classnames0={
  "coca can",
  "beer can",
  "coffee can",
  "2L milk",
  "painkiller",
  "250mm milk",
  "beer bottle",
  "toilet paper",
  "2L water",
  "wine bottle",
  "2L coca bottle",
  "snack vinyl",
}



local send_interval=0.3
local next_interval=3.0
local seq=1

local cameraparam={640,480,343.496368,343.496368,320.5, 240.5}
local xyz_limits={0.25,1.0,   -0.7, 0.7,   0.01,0.32} --xmin xmax ymin ymax zmin zmax

local p_camera={1.3527200222,0,0.6315,-2.3562, 0,1.571359418} --let's hard code it here
local rgb_frame="CAM/camera"
local rgbdata=nil




-- local filename="../Log/Log3527"

local rgb_filename=string.format("%s.png",filename)
local depth_filename=string.format("%s.depth",filename)
local mask_filename=string.format("%s.mask",filename)
local class_filename=string.format("%s.class",filename)



--------------------------------------------------------------------
-- READ DEPTH DATA
local file_depth=io.open(depth_filename,"rb")
local depthdata = file_depth:read("*all")
file_depth:close()
--------------------------------------------------------------------

--------------------------------------------------------------------
-- READ CLASS DATA
local file_class=io.open(class_filename,"r")
local str=file_class:read("*a")
file_class:close()
print("Class:",str)
classnames={}
local count=1
while str do
  local i,j= string.find( str,"%d+")
  if not j then break end
  local numstr=string.gsub(string.sub(str,i,j), "%s+", "")
  print (numstr)
  classnames[count]=classnames0[tonumber( numstr)]
  str=string.sub(str,j+1,-1)
  count=count+1
end
--------------------------------------------------------------------

--------------------------------------------------------------------
-- READ MASK DATA
local f=io.open(mask_filename,"r")
local str=f:read("*a")
f:close()
local mask=ffi.new("int[640*480]")
local count=0
print(#str, #str/25)
for i=1,#str/25 do
  local linestr=string.sub(str,(i-1)*25+1,i*25)
  local a=tonumber(linestr)
  mask[i-1]=a
end
local maskdata=ffi.string(mask,ffi.sizeof(mask))
--------------------------------------------------------------------

--------------------------------------------------------------------
--read RGB data
rospub.imagefrompng(seq,640,480,rgb_filename,2,1,rgb_frame) --send image as png to channel 1
local sub_idx_RGB=rossub.subscribeImage('/CAM/camera/image')
while not rgbdata do
  local w,h,enc,data,ts,frame=rossub.checkImage(sub_idx_RGB)
  if w then
     w_rgb,h_rgb,enc_rgb,rgbdata,tstamp=w,h,enc,data,ts
     rgb_frame=frame
     t_rgb=unix.time()
     print("RGB",enc_rgb,w_rgb,h_rgb)
   end
   unix.usleep(1E6*0.3)
end
--------------------------------------------------------------------




local function find_item_id(name)
  for i=1,#classnames do
    if name==classnames[i] then return i end
  end
  return 0
end

local function update_markers()
  local objnum=wcm.get_objects_num()
  local oxp,oyp,ozp,oyaw,oid,ominw, omaxw=
    wcm.get_objects_xpos(),wcm.get_objects_ypos(),wcm.get_objects_zpos(),
    wcm.get_objects_yaw(),wcm.get_objects_id(),
    wcm.get_objects_min_width(),wcm.get_objects_max_width()

  if objnum>0 then
    local mc=0
    local posx,posy,posz,yaw,orir,orip,oriy={},{},{},{},{},{},{}
    local scales,scalex,scaley,scalez={},{},{},{}
    local names,colors,alphas,types={},{},{},{}

    for i=1,objnum do
      local color=1
      mc=mc+1 --object name marker

      local objfullname="unknown"
      if oid[i]>0 then  objfullname=classnames[oid[i]] end
      types[mc],posx[mc],posy[mc],posz[mc],yaw[mc], names[mc], scales[mc],colors[mc]=
        2, oxp[i],oyp[i],ozp[i]+0.03,0,objfullname, 0.5, color
    end
    -- print(mc, #types,#posx, #posy, #posz, #yaw, #names, #scales, #colors)
    rospub.marker(mc,types,posx,posy,posz,yaw,names,scales,colors)
    posx,posy,posz,orir,orip,oriy={},{},{},{},{},{}
    names,colors,alphas,types={},{},{},{}
    mc=0
    for i=1,objnum do
      local color=1
      mc=mc+1
      types[mc],posx[mc],posy[mc],posz[mc],orir[mc],orip[mc],oriy[mc],scalex[mc],scaley[mc],scalez[mc], names[mc],colors[mc],alphas[mc]=
       5, oxp[i],oyp[i],ozp[i]/2,
        0,0,oyaw[i],
        ominw[i], omaxw[i], ozp[i],  "x", color, 1
    end
    rospub.marker3d(mc,types,   posx,posy,posz,  orir,orip,oriy,  scalex,scaley,scalez,names,colors,alphas)
  end
end










rgbdata_org_mask=yolo2.masktest(
  cameraparam,
  depthdata,
  rgbdata,
  maskdata
)

local det_num, det_names, det_xpos, det_ypos, det_zpos, det_yaw, det_width, det_minwidth, rgbdata2 =yolo2.maskdetect(
  cameraparam,
  rgbdata,
  depthdata,
  p_camera,
  xyz_limits,
  maskdata,
  classnames,
  1
)

local oxp,oyp,ozp,    oyaw,ominw,omaxw,  oid=
  wcm.get_objects_xpos(),wcm.get_objects_ypos(),wcm.get_objects_zpos(),
  wcm.get_objects_yaw(),wcm.get_objects_min_width(),wcm.get_objects_max_width(),
  wcm.get_objects_id()
wcm.set_objects_num(det_num)


for i=1,det_num do
  oxp[i],oyp[i],ozp[i],oyaw[i]=det_xpos[i],det_ypos[i],det_zpos[i],det_yaw[i]
  ominw[i],omaxw[i]=det_minwidth[i],det_width[i]
  oid[i]=find_item_id(det_names[i])
end
wcm.set_objects_xpos(oxp)
wcm.set_objects_ypos(oyp)
wcm.set_objects_zpos(ozp)
wcm.set_objects_yaw(oyaw)
wcm.set_objects_id(oid)
wcm.set_objects_min_width(ominw)
wcm.set_objects_max_width(omaxw)


while true do
  seq=seq+1
  rospub.camerainfo(seq,640,480,1.5)
  rospub.imageonly(seq,640,480,depthdata,2,0,rgb_frame) --send depth as 32FC1
  rospub.imagefrompng(seq,640,480,rgb_filename,2,1,rgb_frame) --send image as png to channel 1
  rospub.imageonly(seq,640,480, rgbdata2, 4,2,rgb_frame)--bgr8
  rospub.imageonly(seq,640,480, rgbdata_org_mask, 4,3,rgb_frame)--bgr8
  unix.usleep(1E6*0.3)
  update_markers()
end


--
-- local function send_log()
--   local f=io.popen("dir ../Log/*.png -1")
--   local str=f:read("*a")
--   --print(str)
--   filelist={}
--   while str do
--     local i,j,line,tail= string.find( str,".png")
--     if not j then break end
--     local linestr=string.gsub(string.sub(str,1,j-4), "%s+", "")
--     filelist[#filelist+1]=linestr
--     str=string.sub(str,j+2,-1)
--   end
--
--   --
--   local index=1
--   while true do
--     local rgb_filename=string.format("%s.png",filelist[index])
--     local depth_filename=string.format("%s.depth",filelist[index])
--     print("Log: "..filelist[index])
--

--

--   -- local rgb_frame="head_rgbd_sensor_rgb_frame"
--     local t0=unix.time()
--     while unix.time()<t0+next_interval do
--       rospub.tf({0,0,0},{0,0,1.57}, "camera0","CAM/camera") --this is provided by webots?

--       unix.usleep(1E6*send_interval)
--       seq=seq+1
--     end
--     index=index+1
--     if index>#filelist then index=1 end
--   end
--
--   -- rospub.tf({0,0,0},{0,0,0}, "map","world")
--   -- rospub.tf({1.35272,0,0.6315},{0,-2.3562,0}, "world","camera0")
--   -- rospub.tf({0,0,0},{0,0,1.57}, "camera0","CAM/camera")
--   -- rospub.tf({0,0,0},{0,0,0}, "camera0","head_rgbd_sensor_rgb_frame")
--
-- end
--
--
-- send_log()
