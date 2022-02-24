local state = {}
state._NAME = ...
local Body  = require'Body'
local t_entry, t_update, t_exit
local timeout = 10.0

local grablist={
  -- {0.30,0.30,0.01  ,0*DEG_TO_RAD} , --xyz theta

  {0.35, 0.45, 0.0125  ,180*DEG_TO_RAD} , --xyz theta
  {0.30,0.35,0.0125  ,90*DEG_TO_RAD} , --xyz theta
  {0.40,0.20,0.0125  ,-90*DEG_TO_RAD} , --xyz theta
  {0.40,0.25,0.0125  ,180*DEG_TO_RAD} , --xyz theta

}

local releaselist={
  {0.43,-0.10,0.0125, 0*DEG_TO_RAD,0*DEG_TO_RAD},  --xyz. pitch, yaw
  {0.43,-0.16,0.0125, 0*DEG_TO_RAD,90*DEG_TO_RAD},  --xyz. pitch, yaw
  {0.49,-0.16,0.0125, 0*DEG_TO_RAD,180*DEG_TO_RAD},  --xyz. pitch, yaw
  {0.49,-0.10,0.0125, 0*DEG_TO_RAD,-90*DEG_TO_RAD},  --xyz. pitch, yaw
}
local is_releasing





function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry

  index=0
  is_releasing=true
  t_check=t_entry
  hcm.set_arm_grabwidth(Config.arm.block_width)
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t

  if (t>t_check+0.1) and hcm.get_arm_motionstate()==0 then
    if index==#grablist and is_releasing then
      return "done"
    else
      if is_releasing then
        index=index+1
        hcm.set_arm_grabxyz({grablist[index][1],grablist[index][2],grablist[index][3]})
        hcm.set_arm_grabrpy({grablist[index][4],90*DEG_TO_RAD,0})
        arm_ch:send'pickup'
        t_check=t
        is_releasing=false
      else
        hcm.set_arm_releasexyz({releaselist[index][1],releaselist[index][2],releaselist[index][3]})
        hcm.set_arm_releaserpy({0,releaselist[index][4],releaselist[index][5]})
        arm_ch:send'release'
        t_check=t
        is_releasing=true
      end
    end
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
