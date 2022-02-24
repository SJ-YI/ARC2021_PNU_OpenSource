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
    hcm.set_motionplan_show_arrow(1-hcm.get_motionplan_show_arrow())



  --   hcm.set_motionplan_targetxya({
  --       -- 0.579, 0.488, 90*DEG_TO_RAD
  --       0.719,-0.127,90*DEG_TO_RAD
  --     })
  --
  --   wcm.set_objects_selected(1)
  --   hcm.set_motionplan_grasptype(0)
  --   hcm.set_motionplan_targettype(5)
  --   hcm.set_motionplan_targetid(9)
  --   hcm.set_motionplan_targetsize(0.08)
  --   hcm.set_motionplan_execute(1)
  -- elseif key_char_lower==("2") then
  --   hcm.set_motionplan_targetxya({
  --       0.8439,0.4884,90*DEG_TO_RAD
  --     })
  --
  --   wcm.set_objects_selected(1)
  --   hcm.set_motionplan_grasptype(0)
  --   hcm.set_motionplan_targettype(3)
  --   hcm.set_motionplan_targetid(3)
  --   hcm.set_motionplan_targetsize(0.15)
  --   hcm.set_motionplan_execute(1)

  elseif key_char_lower==("g") then
    wcm.set_objects_enable(1)
	elseif key_char_lower==("q") then
    hcm.set_motionplan_execute(1)
    hcm.set_motionplan_autorun(1)
  elseif key_char_lower==(" ") then
    hcm.set_motionplan_autorun(0)

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
