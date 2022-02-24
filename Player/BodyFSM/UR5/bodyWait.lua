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
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t
  --we really don't need the bodyinit
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
