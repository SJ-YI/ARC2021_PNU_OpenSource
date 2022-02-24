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
  arm_ch:send'init'
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t

 if (t-t_entry>0.1) and hcm.get_arm_motionstate()==0 then
     return "done" end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
