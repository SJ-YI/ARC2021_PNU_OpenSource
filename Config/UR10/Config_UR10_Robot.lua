
assert(Config, 'Need a pre-existing Config table!')

local vector = require'vector'

Config.nJoint = 6+2
local jointNames={
  "joint1","joint2","joint3","joint4","joint5","joint6",
  "joint_GripperL","joint_GripperR"
}
Config.jointNames = jointNames

local indexArm=1
local nJointArm=6
local indexGripper=7
local nJointGripper=2

Config.parts = {
		Arm = vector.count(indexArm,nJointArm),
    Gripper = vector.count(indexGripper,nJointGripper)
}

local servo={}
servo.direction=vector.new({-1,1,-1,1,1,1,   1,1})
servo.rad_offset=vector.new({0,0,0,0,0,0,  0,0,0})*DEG_TO_RAD
servo.min_rad = vector.ones(Config.nJoint)*-180*DEG_TO_RAD
servo.max_rad = vector.ones(Config.nJoint)*180*DEG_TO_RAD

Config.servo=servo

return Config
