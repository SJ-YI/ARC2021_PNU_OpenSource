local state = {}
state._NAME = ...
local Body  = require'Body'
local t_entry, t_update, t_exit
local timeout = 10.0


function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry


  -- hcm.set_arm_grabxyz({0.30,0.30,0.05})
  hcm.set_arm_grabxyz({0.30,0.30,0.01})

  --Z-Y-X angle
  hcm.set_arm_grabrpy({0*DEG_TO_RAD,90*DEG_TO_RAD, 0*DEG_TO_RAD})
  -- hcm.set_arm_grabrpy({180*DEG_TO_RAD,80*DEG_TO_RAD, 0*DEG_TO_RAD})
  -- hcm.set_arm_grabrpy({180*DEG_TO_RAD,120*DEG_TO_RAD, 0*DEG_TO_RAD})

  -- hcm.set_arm_grabrpy({0,90*DEG_TO_RAD, 0*DEG_TO_RAD})
  arm_ch:send'pickup'
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t

  if (t-t_entry>1.0) and hcm.get_arm_motionstate()==0 then return "done" end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
