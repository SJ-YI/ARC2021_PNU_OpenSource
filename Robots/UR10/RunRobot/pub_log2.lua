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

--for direct maskrcnn testing
rospub.init_custom('pub_log',{"/CAM/range_finder/range_image","/mask_rcnn/input"},{},{"/path"},{"/lowres_map"})


local data= ffi.new("uint8_t[?]",1920*1080*3)
local seq=1

local send_interval=0
local next_interval=10
local num_per_item=1

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
    local t0=unix.time()
    for i=1,num_per_item do
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
end


send_log()
