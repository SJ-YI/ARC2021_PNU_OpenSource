local state = {}
state._NAME = ...
local Body  = require'Body'
local t_entry, t_update, t_exit
local timeout = 10.0
local stage,t_next

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  hcm.set_arm_grabxyz(Config.final_blocks_xyz)
  arm_ch:send'moveto'
  stage=1
  t_next=t_entry+0.1
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t

  if t>t_next and hcm.get_arm_state()==0 then
    if stage==1  then
      t_next=t+1.0
      stage=2
    elseif stage==2 then
      hcm.set_vision_enable(2)
      t_next=t+0.5
      stage=3
    elseif stage==3 then
      hcm.set_vision_enable(0)
      t_next=t+0.5
      stage=4
    elseif stage==4 then
      return "done"
    end
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
