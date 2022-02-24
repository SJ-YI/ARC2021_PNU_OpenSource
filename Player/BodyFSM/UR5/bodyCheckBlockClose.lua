local state = {}
state._NAME = ...
local Body  = require'Body'
local t_entry, t_update, t_exit
local timeout = 10.0
local stage,t_next
local block_selected=0

function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry


  block_selected=wcm.get_cubes2_selected()
  if block_selected>0 then
    local b_sel=block_selected
    local b_xpos=wcm.get_cubes2_xpos()
    local b_ypos=wcm.get_cubes2_ypos()
    local b_zpos=wcm.get_cubes2_zpos()
    local b_rot=wcm.get_cubes2_rot()
    local b_yaw=wcm.get_cubes2_yaw()

    if b_rot[block_selected]>=0 then
      local gyaw=0
      if math.abs(b_yaw[b_sel])<90*DEG_TO_RAD then gyaw=b_yaw[b_sel] end
      local grabpose={b_xpos[b_sel],b_ypos[b_sel],gyaw}
      local viewpose=util.pose_global({-0.024, -0.0145,0},grabpose)
      local grabxyz={viewpose[1],viewpose[2],0.15}      
      hcm.set_arm_grabrpy({0,0,gyaw})
      hcm.set_arm_grabxyz(grabxyz)
      arm_ch:send'moveto'
    else
      print("No target block")
      block_selected=0
    end
  end
  stage=1
  t_next=t_entry+0.1
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t

  if block_selected==0 then return "notarget" end
  if t>t_next and hcm.get_arm_state()==0 then
    if stage==1  then
      t_next=t+1.0
      stage=2
    elseif stage==2 then
      hcm.set_vision_kernelsize({2,20})
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
