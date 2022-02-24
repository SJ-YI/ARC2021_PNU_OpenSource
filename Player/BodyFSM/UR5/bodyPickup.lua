local state = {}
state._NAME = ...
local Body  = require'Body'
local t_entry, t_update, t_exit
local timeout = 10.0
local stage,t_next
local T=require'Transform'

local target_p

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
      local grabxyz,grabrpy={b_xpos[b_sel],b_ypos[b_sel],0.09},{0,0,gyaw}

      hcm.set_arm_grabrpy(grabrpy)
      hcm.set_arm_grabxyz(grabxyz)

      arm_ch:send'moveto'
      hcm.set_hygripper_targetangle(90)

      Body.Kinematics.setup_tool_param(Config.arm.handLength+Config.arm.fingerLength,0,0  ) --finger straight
      local T0=T.trans(grabxyz[1],grabxyz[2],grabxyz[3])*
          T.rotZ(grabrpy[3])*T.rotY(grabrpy[2])*T.rotX(grabrpy[1])
          *T.rotY(90*DEG_TO_RAD)*T.rotX(180*DEG_TO_RAD) --camera position fix
      target_p=T.position6D(T0)
    else
      print("No target block")
      block_selected=0
    end
  end
  stage=1
  t_next=t_entry+1
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t

  if block_selected==0 then return "notarget" end

  if t>t_next and hcm.get_arm_state()==0 then
    if stage==1  then
      local target_p2=util.shallow_copy(target_p)
      target_p2[3]=target_p2[3]-0.05
      hcm.set_arm_pTarget(target_p2)
      hcm.set_arm_execute(3) --move IK
      t_next=t+1.0
      stage=2

    elseif stage==2 then
      hcm.set_hygripper_targetvel(-15000)
      hcm.set_hygripper_execute(2)
      t_next=t+1.0
      stage=3

    elseif stage==3 then
      hcm.set_arm_pTarget(target_p)
      hcm.set_arm_execute(3) --move IK
      t_next=t+1.0
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
