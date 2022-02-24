assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'
local fsm = {}
-- Update rate in Hz
fsm.update_rate = 100

-- Which FSMs should be enabled?
fsm.enabled = {
  -- Head = true,
   Body = true,
   Arm=true,
   Motion = true,
}

fsm.select = {
  -- Head = 'N1Bot',
  Arm="UR5E_ActiveGripper",
  Body = 'Zenga',
  Motion = 'UR5',
}

if IS_WEBOTS then
  fsm.Motion = {
    {'motionIdle', 'movejoint', 'motionMoveJoint'},
    {'motionIdle', 'movelinear', 'motionMoveLinear'},
    {'motionMoveJoint','done','motionIdle'},
    {'motionMoveLinear','done','motionIdle'},

    {'motionIdle', 'movevelocity', 'motionMoveVelocity'},
    {'motionMoveJoint', 'movevelocity', 'motionMoveVelocity'},
    {'motionMoveLinear', 'movevelocity', 'motionMoveVelocity'},
  }
  fsm.Arm = {
    {'armIdle', 'init', 'armInitWebots'},
    {'armInitWebots', 'done', 'armIdle'},
    {'armIdle', 'moveto', 'armMoveto'},
    {'armMoveto', 'done', 'armIdle'},
  }

else
  fsm.Motion = {
    {'motionUR5', 'done', 'motionUR5'},
  }
  fsm.Arm = {
    {'armIdle', 'init', 'armInit'},
    {'armInit', 'testdirect', 'armTestDirect'},
    {'armInit', 'done', 'armIdle'},

    {'armIdle', 'pickup', 'armPickup'},
    {'armIdle', 'release', 'armRelease'},
    {'armIdle', 'moveto', 'armMoveto'},
    {'armMoveto', 'done', 'armIdle'},
    {'armPickup', 'done', 'armIdle'},
    {'armPickup', 'init', 'armInit'},
    {'armRelease', 'done', 'armIdle'},
  }
end


fsm.Body = {
  {'bodyIdle', 'init', 'bodyInit'},
  {'bodyInit', 'done', 'bodyWait'},
  -- {'bodyInit', 'done', 'bodyCheckBlocks'},
  -- {'bodyInit', 'done', 'bodyCheckTarget'},
  --
  {'bodyWait', 'checkblock', 'bodyCheckBlocks'},
  {'bodyWait', 'checktarget', 'bodyCheckTarget'},
  -- {'bodyWait', 'checkblockclose', 'bodyCheckBlockClose'},
  --
  {'bodyWait', 'pickup', 'bodyPickup'},
  -- {'bodyWait', 'release', 'bodyRelease'},
  --
  {'bodyCheckBlocks', 'done', 'bodyWait'},
  -- {'bodyCheckBlocks', 'checktarget', 'bodyCheckTarget'},
  -- {'bodyCheckBlocks', 'blockclose', 'bodyCheckBlockClose'}, --fix upright blocks
  --
  --
  {'bodyCheckTarget', 'done', 'bodyWait'},
  {'bodyCheckTarget','checkblock', 'bodyCheckBlocks'},
  -- {'bodyCheckTarget', 'start', 'bodyCheckBlockClose'},
  -- {'bodyCheckTarget', 'cleantarget', 'bodyCleanTarget'},
  --
  -- {'bodyCheckTarget', 'checkblock', 'bodyCheckBlocks'},
  --
  -- {'bodyCheckBlockClose', 'notarget', 'bodyInit'},
  -- {'bodyCheckBlockClose', 'done', 'bodyWait'},
  -- {'bodyCheckBlockClose', 'checkblock', 'bodyCheckBlocks'},
  --
  -- -- {'bodyCheckBlockClose', 'pickup', 'bodyPickup'},
  --
  --
  -- {'bodyCheckBlockClose', 'pickup', 'bodyPickup'},
  -- {'bodyCheckBlockClose', 'pickup2', 'bodyPickup'},
  --
  -- -- {'bodyPickupAndRelease', 'done', 'bodyCheckBlockClose'},
  --
  {'bodyPickup', 'done', 'bodyWait'},
  -- {'bodyPickup', 'done', 'bodyRelease'},
  -- {'bodyRelease', 'done', 'bodyCheckBlockClose'},
  --
  -- {'bodyWait', 'testrelease', 'bodyTestRelease'},
  -- {'bodyTestRelease', 'init', 'bodyInit'},
  --
  -- {'bodyWait', 'testpickup', 'bodyTestPickup'},
  -- {'bodyTestPickup', 'init', 'bodyInit'},
  --
  -- {'bodyWait', 'testclose', 'bodyTestClose'},
  -- {'bodyTestClose', 'init', 'bodyInit'},
  --
  -- {'bodyTestClose', 'testclose', 'bodyTestClose'},
  -- {'bodyTestClose', 'checkblock', 'bodyCheckBlocks'},
  --
  -- {'bodyWait', 'cleantarget', 'bodyCleanTarget'},
  -- {'bodyCleanTarget', 'done', 'bodyInit'},
  -- {'bodyCleanTarget', 'checkblock', 'bodyCheckBlocks'},

}












fsm.libraries = {}

Config.arm={}

------------ZEUS configs


Config.arm.qInit=vector.new({0,0,90,-90,-90,-180})*DEG_TO_RAD

Config.initial_blocks_xyz={0,0.40,0.30}
-- Config.target_xyz={0,-0.40, 0.30}

Config.target_xyz={0,-0.60, 0.50}


Config.ground_offset_z_vision=0
Config.ground_offset_z=0 --FOR PICKUP


--webots
-- Config.arm.max_angular_vel = vector.new({180,180,180,180,180,180})*DEG_TO_RAD*1.0
Config.arm.max_angular_vel = vector.new({90,90,90,180,180,180})*DEG_TO_RAD*1.0
-- Config.arm.max_gripper_vel = {0.02,0.02}
Config.arm.max_gripper_vel = {0.05,0.05}
------------ZEUS configs


Config.pickupHeight1 = 0.05 --move directly to this above the pickup pos
Config.pickupHeight2 = 0.09 --move up this amount after pickup

Config.pickupDelay1 = 0.1 --wait for this amount before gripper action
-- Config.pickupDelay2 = 0.3 --wait for this amount after gripper action
Config.pickupDelay2 = 0.7 --wait for this amount after gripper action


Config.kernelsize_blocks={0,10} --works well with webots







--Force wrist outside setting

Config.wrist_mode=4 --FORCE wrist outside
Config.yaw_center=-90*DEG_TO_RAD



-- Config.close_view_z = 0.15
Config.close_view_z = 0.175
Config.close_view_offset_mag = 0.025

--New hinge finger hand
Config.arm.handLength = 0.107
Config.arm.fingerLength=0.050 --NEW FINGER!!!

Config.block_size=0.025
Config.pickup_depth = 0.005
Config.camera_offsets={0.073,-0.0155, -0.024}

Config.pickup_width_offset=10
Config.release_width_offset=-5





-- Config.ReleaseHeight1 = 0.1
Config.ReleaseHeight1 = 0.12
Config.ReleaseHeight2 = 0.09 --for continuous motion

Config.releaseDelay1 = 0.1 --wait for this amount before gripper action
Config.releaseDelay2 = 0.2 --wait for this amount before gripper action

--try reduce this
Config.blockDetectDelay = 0.3
Config.blockDetectCloseDelayFirst = 2.0
Config.blockDetectCloseDelay = 0.5
Config.targetDetectDelay = 0.3

--How many images we detect to be sure
Config.block_detect_num=5
Config.blockclose_detect_num=1
Config.target_detect_num=1





Config.armvellimit={1.5,2}
Config.armlvellimit={0.5,0.2} --for lowering the arm
Config.armlvellimit2={0.5,0.2} --for raising the arm
Config.armlvelslow={1.0,1.6, 0.2, 0.1}






Config.final_blocks_xyz={0, -0.50, 0.30}

--coex
  Config.assemble_blocks_pose={0.37,-0.227,0 }
  Config.regrasp_blocks_pose={0.55,0.125,0}

  Config.regrasp_blocks_pose2={0.325,0.16,0} --for red
  Config.regrasp_blocks_pose3={0.4125,0.16,0} --for orange
  Config.regrasp_blocks_pose4={0.50,0.16,0} --for yellow

  Config.cleantarget_xyz={0.415+0.169,0.40,0.30}

  Config.cleanHeight=0.15
  Config.cleanHeight2=0.02
  Config.cleanHeight2=0.045 --fingertip at 3rd row center

  --tadpole clearing
  Config.cleantarget_x_mag = 0.12
  Config.cleanHeight2=0.035 --fingertip at 3rd row

  Config.camera_calibration_angle={110*DEG_TO_RAD, -1.625*DEG_TO_RAD}

  Config.camera_params={320,252,610}--461 407/ 461 408




--COEX
  Config.camera_calibration_angle={100*DEG_TO_RAD, -2*DEG_TO_RAD}

  Config.camera_params={315,258,610}--446 403/447 403

  Config.release_z_margin = 0.003
  Config.release_z_margin = 0.002 --COEX
  Config.release_z_margin = 0.001 --COEX

  Config.side_release_offset_z = 0.006
  Config.side_release_offset_y = 0.003 --little far away
  Config.side_release_safety_margin = 0.004

  Config.blockDetectDelay = 1.0
  Config.blockDetectCloseDelayFirst = 3.0
  Config.blockDetectCloseDelay = 2.0
  Config.armlvellimit={0.2,0.1}

  Config.block_pos_error_max=0.02
  Config.block_pos_error_max2=0.05 --for regrasp

  Config.assemble_margin = 0.0025 --put this amount of gaps between blocks
  Config.assemble_margin_func={0,1,2,2.8,3.6,4.4,5.2, 6.0}
  Config.force_target=true

--FASTER SETTING
  Config.blockDetectCloseDelayFirst = 3.0
  Config.blockDetectCloseDelay = 1.5


  Config.block_detect_num=8




if IS_WEBOTS then
  Config.assemble_blocks_pose={0.37,-0.227,0 }
end



Config.fsm = fsm

-- Add all FSM directories that are in Player
for _,sm in ipairs(fsm.enabled) do
  local pname = {HOME, '/Player/', sm, 'FSM', '/?.lua;', package.path}
  package.path = table.concat(pname)
end

return Config
