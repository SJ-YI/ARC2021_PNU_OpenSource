#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
local ok = pcall(dofile,'../fiddle.lua')
if not ok then dofile'fiddle.lua' end
require'wcm'
require'hcm'
local unix=require'unix'
local vector=require'vector'


local t0
local outfile
local outfilename



local function entry()
  t0=unix.time()
  hcm.set_voice_execute(0)
  -- local date_str=os.date('%m%d_%H%M')
  -- outfilename="../Log/"..date_str..".txt"
end

-- local function add_log(str)
--   if wcm.get_robot_rostime_gamestart()>0 then
--     local t_ros=wcm.get_robot_rostime()
--     local t_elapsed=t_ros-wcm.get_robot_rostime_gamestart()
--     local t_min=math.floor(t_elapsed/60)
--     local t_sec=math.floor(t_elapsed%60)
--     local t_ms=math.floor((t_elapsed*1000)%1000)
--
--     local logstr=string.format("[%2d:%2d : %3d] %s",t_min,t_sec,t_ms,str)
--     print(logstr)
--     outfile=io.open(outfilename,"a")
--     outfile:write(logstr.."\n")
--     outfile:close()
--   end
-- end


local function check_msg()
  local t=unix.time()

  local voice=hcm.get_voice_execute()
  if not voice then
    print("VOICE NIL!!")
    hcm.set_voice_execute(0)
  end
  if voice>0 then
    hcm.set_voice_execute(0)
    local str=hcm.get_voice_str()
    print("Voice: "..str)
    os.execute("espeak '"..str.."'")
    -- add_log(str)
  end

  -- local voicelog=hcm.get_voicelog_execute()
  -- if voicelog>0 then
  --   hcm.set_voicelog_execute(0)
  --   local str=hcm.get_voicelog_str()
  --   add_log(str)
  -- end
end

local function update()
  check_msg()
end

if ... and type(...)=='string' then --webots handling
  WAS_REQUIRED = true
  return {entry=entry, update=update, exit=nil}
end

local count,tpassed,t_last_debug,max_t_passed=0,0,unix.time(),0
local running = true
local key_code
entry()


while running do
  local t0=unix.time()
  update()
  local t1=unix.time()
  tpassed=tpassed+t1-t0
  max_t_passed=math.max(t1-t0,max_t_passed)
  count=count+1
  unix.usleep(1E6*0.01)
  if count>1000 then
    local t=unix.time()
    local t_total=t-t_last_debug
    print(string.format("Voice wizard, average %.1f fps / %.2f ms, peak %.2f ms",
      count/t_total,tpassed/count*1000,max_t_passed*1000))
    tpassed,count,t_last_debug,max_t_passed=0,0,t,0
  end
end
