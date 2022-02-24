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


rospub.init_custom('pub_log',{"/CAM/range_finder/range_image","/CAM/camera/image"},{},{"/path"},{"/lowres_map"})

-- --FOR DIRECT YOLO TESTING
-- rospub.init_custom('pub_log',{"/CAM/range_finder/range_image","/pnu/rgb_rect_raw"},{},{"/path"},{"/lowres_map"})


local data= ffi.new("uint8_t[?]",1920*1080*3)
local seq=1

-- local send_interval=0.5
-- local send_interval=1.5
-- local next_interval=3.0

local send_interval=0.2
-- local next_interval=3.0
local next_interval=5.0


local function send_log()
  local f=io.popen("dir ../Log/*.png -1")
  local str=f:read("*a")
  --print(str)
  filelist={}
  while str do
    local i,j,line,tail= string.find( str,".png")
    if not j then break end
    local linestr=string.gsub(string.sub(str,1,j-4), "%s+", "")
    filelist[#filelist+1]=linestr
    str=string.sub(str,j+2,-1)
  end

  print("Total images:",#filelist)
  local index=1
  while true do
     -- index=10

    local rgb_filename=string.format("%s.png",filelist[index])
    local depth_filename=string.format("%s.depth",filelist[index])
    print("Log: "..filelist[index])
    local depthdata=nil
    local file_depth=io.open(depth_filename,"rb")
    if file_depth then
      depthdata = file_depth:read("*all")
      file_depth:close()
    end


    local rgb_frame="CAM/camera"
  -- local rgb_frame="head_rgbd_sensor_rgb_frame"
    local t0=unix.time()

    for i=1,3 do
      rospub.tf({0,0,0},{0,0,1.57}, "camera0","CAM/camera") --this is provided by webots?
      rospub.camerainfo(d_seq,640,480,1.5)
      if depthdata then
        rospub.imageonly(seq,640,480,depthdata,2,0,rgb_frame) --send depth as 32FC1
      end
      rospub.imagefrompng(seq,640,480,rgb_filename,2,1,rgb_frame) --send image as png to channel 1
      unix.usleep(1E6*send_interval)
      seq=seq+1
    end
    unix.usleep(1E6*next_interval)
    index=index+1
    if index>#filelist then index=1 end
  end

  -- rospub.tf({0,0,0},{0,0,0}, "map","world")
  -- rospub.tf({1.35272,0,0.6315},{0,-2.3562,0}, "world","camera0")
  -- rospub.tf({0,0,0},{0,0,1.57}, "camera0","CAM/camera")
  -- rospub.tf({0,0,0},{0,0,0}, "camera0","head_rgbd_sensor_rgb_frame")

end


send_log()
