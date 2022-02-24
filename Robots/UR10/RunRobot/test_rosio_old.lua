#!/usr/bin/env luajit
-- (c) 2014 Team THORwIn
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end
local WAS_REQUIRED
require'libTask'

-- local rospub = require 'rospub'
-- local rossub = require 'rossub'
-- rospub.init('test_publisher')
local unix=require'unix'
local t_last_pressed=unix.time()+0.1

local grab_yaw=0

local function update(key_code)
  local t=unix.time()
	if type(key_code)~='number' or key_code==0 then return end

  if t-t_last_pressed<0.5 then return end
  t_last_pressed=t
	local key_char = string.char(key_code)
	local key_char_lower = string.lower(key_char)

	if key_char_lower==("1") then
    hcm.set_motionplan_targettype(1)
    print("OBJECT TYPE 1")
  elseif key_char_lower==("2") then
    hcm.set_motionplan_targettype(2)
    print("OBJECT TYPE 2")
  elseif key_char_lower==("3") then
    hcm.set_motionplan_targettype(3)
    print("OBJECT TYPE 3")
  elseif key_char_lower==("4") then
    hcm.set_motionplan_targettype(4)
    print("OBJECT TYPE 4")
  elseif key_char_lower==("5") then
    hcm.set_motionplan_targettype(5)
    print("OBJECT TYPE 5")


  elseif key_char_lower==("-") then
    hcm.set_motionplan_graspwidth(0.8)
    print("semi grasp")
  elseif key_char_lower==("=") then
    hcm.set_motionplan_graspwidth(1.1)
    print("full grasp")

  elseif key_char_lower==("g") then
    wcm.set_objects_enable(1)

  elseif key_char_lower==("[") then
    local num=wcm.get_objects_num()
    local sel=wcm.get_objects_selected()
    sel=sel-1
    if sel<1 then sel=num end
    wcm.set_objects_selected(sel)
    print("sel:",sel)

  elseif key_char_lower==("]") then
    local num=wcm.get_objects_num()
    local sel=wcm.get_objects_selected()
    sel=sel+1
    if sel>num then sel=1 end
    wcm.set_objects_selected(sel)
    print("sel:",sel)

  elseif key_char_lower==("i") then
    grab_yaw=180*DEG_TO_RAD
    print("Backward grab")

  elseif key_char_lower==(",") then
    grab_yaw=0*DEG_TO_RAD
    print("Forward grab")
  elseif key_char_lower==("l") then
    grab_yaw=-90*DEG_TO_RAD
    print("Right grab")
  elseif key_char_lower==("j") then
    grab_yaw=90*DEG_TO_RAD
    print("Left grab")

  elseif key_char_lower==("p") then
    local num=wcm.get_objects_num()
    local sel=wcm.get_objects_selected()
    local objx, objy, objz = wcm.get_objects_xpos(),wcm.get_objects_ypos(),wcm.get_objects_zpos()
    if num>0 then
      print(num,sel)
      print(objx[sel],objy[sel])
      hcm.set_motionplan_targetxya({objx[sel], objy[sel], grab_yaw })
      hcm.set_motionplan_execute(1)
    end
	elseif key_char_lower==("q") then
    hcm.set_motionplan_execute(1)
  end


  mcm.set_walk_vel(targetvel_new)

end

if ... and type(...)=='string' then --webots handling
  WAS_REQUIRED = true
  return {entry=nil, update=update, exit=nil}
end



local getch = require'getch'
local running = true
local key_code
while running do
  key_code = getch.nonblock()
  update(key_code)
  unix.usleep(1E6*0.02)
end
