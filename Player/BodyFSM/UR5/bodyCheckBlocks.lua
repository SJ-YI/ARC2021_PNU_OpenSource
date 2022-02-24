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
  hcm.set_arm_grabxyz(Config.initial_blocks_xyz)
  hcm.set_arm_grabrpy({0,0,0})
  arm_ch:send'moveto'
  stage=1
  t_next=t_entry+0.1
  wcm.set_cubes2_rot({-1,-1,-1,-1,-1,-1,-1})--reset block data
  hcm.set_hygripper_targetangle(0)
  hcm.set_hygripper_targetpos(35)
  hcm.set_hygripper_execute(1)
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t

  if t>t_next and hcm.get_arm_state()==0 then
    if stage==1  then
      hcm.set_vision_kernelsize({0,20})
      t_next=t+1.0
      stage=2
    elseif stage==2 then

      hcm.set_vision_enable(1)
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
  hcm.set_vision_enable(0)
  print(state._NAME..' Exit' )
end

return state
