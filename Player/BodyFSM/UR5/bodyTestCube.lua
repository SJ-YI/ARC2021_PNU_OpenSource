local state = {}
state._NAME = ...
local Body  = require'Body'
local T=require'Transform'
local t_entry, t_update, t_exit
local timeout = 10.0



--pitch-yaw-roll
local grablist={
     {1,   0,0,1,    -180*DEG_TO_RAD} ,
     {2,   0,-1,1,    -180*DEG_TO_RAD} ,
     {5,   0,0,1,    0*DEG_TO_RAD} ,
     {4,   0,0,1,    90*DEG_TO_RAD} , --camera to the right
     {6,   0,1,0,    -90*DEG_TO_RAD} , --camera to the left
     {7,   0,0,1,    -180*DEG_TO_RAD} , --camera to the bottom
     {3,   1,0,0,    0*DEG_TO_RAD} , --camera to the bottom


}

local releaselist={ --yaw pitch
    {1, -1, 0.5,   180*DEG_TO_RAD, -90*DEG_TO_RAD}, --block to the bottom
    -- {-1, 0, 1,   90*DEG_TO_RAD, -90*DEG_TO_RAD}, --block to the left
    -- {-1,-1, 1.5,   0*DEG_TO_RAD, -90*DEG_TO_RAD}, --block to the up
    -- {1, 1, 1.5,   -90*DEG_TO_RAD, -90*DEG_TO_RAD}, --block to the right
    -- {0, 1, 2.5,   0*DEG_TO_RAD, -90*DEG_TO_RAD}, --block to the up
    -- {1, 0, 3,   -90*DEG_TO_RAD, -90*DEG_TO_RAD}, --block to the right
    -- {0, 0, 3,   0*DEG_TO_RAD, 0*DEG_TO_RAD},

    {-1, 0, 1.5,   90*DEG_TO_RAD, -90*DEG_TO_RAD}, --block to the left
    {-1,-1, 2.5,   0*DEG_TO_RAD, -90*DEG_TO_RAD}, --block to the up
    {1, 1, 2.5,   -90*DEG_TO_RAD, -90*DEG_TO_RAD}, --block to the right
    {0, 1, 3.5,   0*DEG_TO_RAD, -90*DEG_TO_RAD}, --block to the up
    {1, 0, 4.5,   -90*DEG_TO_RAD, -90*DEG_TO_RAD}, --block to the right
    {0, 0, 3.5,   0*DEG_TO_RAD, 0*DEG_TO_RAD},




}
local offsets={
  {0,-1},
  {-1,1},
  {-1,-1},
  {1,0},
  {-1,1},
  {0,-1},
  {-1,0},
}
local offsets={
  {3,0},--1
  {0,2},--2
  {0,1},--3
  {4,1},--4
  {2,0},--5
  {1,2},--6
  {3,0},--7

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


  -- local block_num=wcm.get_cubes_num()
  local block_num=7
  local block_xpos=wcm.get_cubes_xpos()
  local block_ypos=wcm.get_cubes_ypos()
  local block_zpos=wcm.get_cubes_zpos()
  local block_r=wcm.get_cubes_r()
  local block_p=wcm.get_cubes_p()
  local block_y=wcm.get_cubes_y()

  local xoffsets,yoffsets={},{}
  for i=1,block_num do
    xoffsets[i],yoffsets[i]=offsets[i][1],offsets[i][2]
  end

  wcm.set_cubes_xoffsets(xoffsets)
  wcm.set_cubes_yoffsets(yoffsets)


  print(block_num)
  for i=1,block_num do
    print(string.format("Block %d pose:%.2f %.2f %.2f (%.3f %.3f %.3f)",
      i,
      block_xpos[i],
      block_ypos[i],
      block_zpos[i],
      block_r[i]/DEG_TO_RAD,
      block_p[i]/DEG_TO_RAD,
      block_y[i]/DEG_TO_RAD
    ))
  end
end

function state.update()
  local t  = Body.get_time()
  local dt = t - t_update
  t_update = t

  if (t>t_check+0.1) and hcm.get_arm_motionstate()==0 then

    local block_xpos=wcm.get_cubes_xpos()
    local block_ypos=wcm.get_cubes_ypos()
    local block_zpos=wcm.get_cubes_zpos()
    local block_r=wcm.get_cubes_r()
    local block_p=wcm.get_cubes_p()
    local block_y=wcm.get_cubes_y()
    local block_xoffset=wcm.get_cubes_xoffsets()
    local block_yoffset=wcm.get_cubes_yoffsets()

    if Config.motion_test then
      -- block_xpos={0.375,0.375,0.45,0.45,0.55,0.525,0.30}
      block_xpos={0.395,0.395,0.47,0.47,0.57,0.545,0.32}
      block_ypos={0.36, 0.26, 0.36, 0.26 ,0.36, 0.26, 0.36}
      block_zpos={0.0125,0.0125,0.0125,0.0125,0.0125,0.0125,0.0125}
      block_r={0,0,0,0,0,0,0}
      block_p={0,0,0,0,0,0,0}
      block_y={0,0,0,0,0,0,0}
    end






    if index==#grablist and is_releasing then
      return "done"
    else
      if is_releasing then

        index=index+1
        block_id=grablist[index][1]


--FOR TESTING!!!!!!!!!!!!!
        local bx,by,bz,brx,bry,brz=
        block_xpos[block_id],
        block_ypos[block_id],
        block_zpos[block_id],
        block_r[block_id],
        block_p[block_id],
        block_y[block_id]

        local T1=T.trans(bx,by,bz)
            *T.rotY(bry)*T.rotZ(brz)*T.rotX(brx)
            *T.trans(
              grablist[index][2]*Config.arm.block_width,
              grablist[index][3]*Config.arm.block_width,
              grablist[index][4]*Config.arm.block_width
            )
            *T.rotY(math.pi/2)
            *T.rotX(grablist[index][5])


        local p=T.position6D(T1)
        print(string.format("Block P:%.2f %.2f %.4f/ %.3f %.3f %.3f",
          p[1],p[2],p[3],p[4]/DEG_TO_RAD,p[5]/DEG_TO_RAD,p[6]/DEG_TO_RAD
        ))

        hcm.set_arm_grabxyz({p[1],p[2],p[3]})
        hcm.set_arm_grabrpy({p[4],p[5],p[6]})
        arm_ch:send'pickup'
        t_check=t
        is_releasing=false
      else
        block_id=grablist[index][1]


--FOR TESTING!!!!!!!!!!!!!
        local v_offset=0
        if Config.motion_test then v_offset=Config.finger_v_offset or 0.02 end



        local T1=T.transform6D(Config.pReleaseCenter)
            *T.trans(
              releaselist[index][1]*Config.arm.block_width + Config.arm.block_slack*block_xoffset[block_id],
              releaselist[index][2]*Config.arm.block_width + Config.arm.block_slack*block_yoffset[block_id],
              releaselist[index][3]*Config.arm.block_width+v_offset
            )
            *T.rotZ(releaselist[index][4])
            *T.rotY(releaselist[index][5])
            *T.rotY(math.pi/2)
        local p=T.position6D(T1)

        if releaselist[index][3]>=1 then --add finger thickness offset
          -- p[3]=p[3]+Config.arm.finger_width/2 + Config.arm.block_slack
          -- p[3]=p[3]+Config.arm.finger_width/2
          -- p[3]=p[3]+Config.arm.finger_width
        end

        print("Release P:",unpack(p))

        hcm.set_arm_releasexyz({p[1],p[2],p[3]})
        hcm.set_arm_releaserpy({p[4],p[5],p[6]})
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
