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

  wcm.set_robot_pose({0,0,0})
  wcm.set_robot_pose_odom({0,0,0})
--  wcm.set_slam_pose({0,0,0})

  hcm.set_vision_enable(0)

end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  --we really don't need the bodyinit
  -- if (t-t_entry>1.0) then return "init" end
  if (t-t_entry>1.0) then
    print("going to init")
    return "init"
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
