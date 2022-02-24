local state = {}
state._NAME = ...
local Body  = require'Body'
local T=require'Transform'
local t_entry, t_update, t_exit
local timeout = 10.0
local q0
local pTarget={0.6, 0.0, 0.10,  180*DEG_TO_RAD, 0, 180*DEG_TO_RAD}
local qTarget=Body.Kinematics.inverse_arm(pTarget,{0,-45,180,0,45,0})
local max_angular_vel=vector.ones(6)*50*DEG_TO_RAD

local pickup_stage

local cube_indices={1,2,5,4,6,7,3}
-- local cube_indices={3}

local cube_indices={}
local grab_widths={}
local pickup_xyzypy={}
local release_xyzypy={}
local xoffsets=vector.zeros(7)
local yoffsets=vector.zeros(7)

local function load_movement()

-- local p_ypy = T.to_zyz( T.rotZ(0*DEG_TO_RAD)*T.rotX(-90*DEG_TO_RAD))
-- print(unpack(p_ypy))


  wcm.set_cubes_pickupseq({

  1, 1,   0, 0, 1,    3.141592653589,0, -1.5707963267949,
          1, -1, -0.5, 0, 1.5707963267949, 1.5707963267949,

  2, 1,    0, -1, 1,      -90*DEG_TO_RAD,0,0,
            -1, 0, 0,       -90*DEG_TO_RAD, 90*DEG_TO_RAD,-90*DEG_TO_RAD,

  5, 1,    0, 0, 1,  0,0,90*DEG_TO_RAD,
          -1, -1, 0, 0,-90*DEG_TO_RAD,90*DEG_TO_RAD,

  4, 1,    0, 0, 1,  0,0,0,
           1, 1, 0, 90*DEG_TO_RAD,90*DEG_TO_RAD,-90*DEG_TO_RAD,

  6, 1,    0, 1, 0,   0, 0, 0*DEG_TO_RAD,
           0, 1, 1,   -180*DEG_TO_RAD, 90*DEG_TO_RAD, 90*DEG_TO_RAD,

  7, 1,    0, 0, 1,   0,0,-90*DEG_TO_RAD,
           1, 0, 2,   90*DEG_TO_RAD,90*DEG_TO_RAD,-90*DEG_TO_RAD,

  3, 1,    1, 0, 0,     0, 0, -90*DEG_TO_RAD,
          0, 0, 2,    -90*DEG_TO_RAD, 0, 0,
  -- 3, 3,    0, 0, 0,     0, 0, -90*DEG_TO_RAD,
  --         -1, 0, 2,    -90*DEG_TO_RAD, 0, 0,

  })

--1: 0
--5: -1
--


  local pickupseq=wcm.get_cubes_pickupseq()
  for i=1,7 do
    local cur_movement=vector.slice(pickupseq,(i-1)*14+1,(i-1)*14+14)
    cube_indices[i]=cur_movement[1]
    grab_widths[i]=cur_movement[2]
    pickup_xyzypy[i]=vector.slice(cur_movement,3,8)
    release_xyzypy[i]=vector.slice(cur_movement,9,14)
  end

  local xoffsets=wcm.get_cubes_xoffsets()
  local yoffsets=wcm.get_cubes_yoffsets()
  for i=1,#xoffsets do
    print("Offset:"..xoffsets[i]..","..yoffsets[i]..")" )
  end

end


local function get_grabpose(seq_index,index)
  print("index:",seq_index,index)
  local cs=Config.cubes.size
  local c_xpos,c_ypos,c_zpos=wcm.get_cubes_xpos(),wcm.get_cubes_ypos(),wcm.get_cubes_zpos()
  local c_r,c_p,c_y=wcm.get_cubes_r(),wcm.get_cubes_p(),wcm.get_cubes_y()
  -- local xyz0=vector.new({c_xpos[index],c_ypos[index],c_zpos[index] + Config.cubes.size/2 })
  -- local rpy0={180*DEG_TO_RAD, 0, 180*DEG_TO_RAD+c_y[index]}
  local pickup=pickup_xyzypy[seq_index]

print(unpack(pickup))
  print("Pickup offset;",pickup[1],pickup[2],pickup[3])
  -- local T1 = T.transform6D({
  --   c_xpos[index],c_ypos[index],c_zpos[index],c_r[index],c_p[index],c_y[index]})

  local T1 = T.trans(c_xpos[index],c_ypos[index],c_zpos[index])
    -- *T.rotZ(c_y[index])
    -- *T.rotY(c_p[index])
    -- *T.rotX(c_r[index])
    --TODO: check webots rpy sequence
    *T.rotY(c_p[index])
    *T.rotZ(c_y[index])
    *T.rotX(c_r[index])


    *T.trans(pickup[1]*cs,pickup[2]*cs,pickup[3]*cs)
    *T.rotZ(pickup[4])
    *T.rotY(pickup[5])
    *T.rotZ(pickup[6])
    *T.rotY(math.pi)
   p=T.position6D(T1)



   return {p[1],p[2],p[3]}, {p[4],p[5],p[6]}
end

local function get_releasepose(seq_index,index)
  local cs=Config.cubes.size
  local slack=Config.cubes.slack
  local r_center = Config.cube_release_center
  local release = release_xyzypy[seq_index]
  local xoffset=wcm.get_cubes_xoffsets()[index]
  local yoffset=wcm.get_cubes_yoffsets()[index]

print("release:",unpack(release))

  local T1 = T.trans(
    r_center[1]+release[1]*cs+slack*xoffset,
    r_center[2]+release[2]*cs+slack*yoffset,
    r_center[3]+(release[3]+1)*cs,       0,0,0)
    *T.rotZ(release[4])
    *T.rotY(release[5])
    *T.rotZ(release[6])
    *T.rotY(math.pi)
  p=T.position6D(T1)
  return {p[1],p[2],p[3]}, {p[4],p[5],p[6]}
end


function state.entry()
  print(state._NAME..' Entry' )
  local t_entry_prev = t_entry
  t_entry = Body.get_time()
  t_update = t_entry
  pickup_stage=1
  load_movement()
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t

  local arm_state=hcm.get_arm_state()
  -- print(arm_state)
  if arm_state==0 then
    arm_ch:send'init'
    return
  elseif arm_state==2 then --initialized state
     if pickup_stage<=#cube_indices then
       local xyz,rpy=get_grabpose(pickup_stage, cube_indices[pickup_stage])
       print("Block pickup: ",cube_indices[pickup_stage])

       hcm.set_arm_grabxyz(xyz)
       hcm.set_arm_grabrpy(rpy)
       hcm.set_arm_grabwidth(Config.cubes.size*grab_widths[pickup_stage])
       arm_ch:send'pickup'
     end
  elseif arm_state==4 then --picked up something
    local xyz,rpy=get_releasepose(pickup_stage, cube_indices[pickup_stage])
    hcm.set_arm_releasexyz(xyz)
    hcm.set_arm_releaserpy(rpy)
    arm_ch:send'release'
    pickup_stage=pickup_stage+1
  elseif arm_state==6 then --release done!
     hcm.set_arm_state(2)
  end
end

function state.exit()
  print(state._NAME..' Exit' )
end

return state
