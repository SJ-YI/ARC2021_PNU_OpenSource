--------------------------------
-- Joint Communication Module
-- (c) 2013 Stephen McGill
--------------------------------
local Config = Config or require'Config'
assert(Config, 'DCM requires a config, since it defines joints!')
local nJoint = Config.nJoint
local memory = require'memory'
local vector = require'vector'


local shared_data = {}
local shared_data_sz = {}

-- Sensors from the robot
shared_data.sensor = {}

--setup sensor variables
shared_data.sensor.position = vector.zeros(nJoint)


-- Sensors from the robot
shared_data.tsensor = {}


-- These should not be tied in with the motor readings,
-- so they come after the read/tread setup
-- Raw inertial readings
-- shared_data.sensor.accelerometer = vector.zeros(3)
-- shared_data.sensor.gyro          = vector.zeros(3)
-- shared_data.sensor.magnetometer  = vector.zeros(3)
-- shared_data.sensor.imu_t  = vector.zeros(1) --we timestamp IMU so that we can run IMU later than state wizard
-- shared_data.sensor.imu_t0  = vector.zeros(1) --run_imu start time
-- shared_data.sensor.rpy           = vector.zeros(3) -- Filtered Roll/Pitch/Yaw
-- shared_data.sensor.battery       = vector.zeros(1) -- Battery level (in volts)
-- shared_data.sensor.compass       = vector.zeros(3)


--  Write to the motors
shared_data.actuator = {}

-- Additional memory variables (to make old dcm code work)
shared_data.actuator.gainLevel = vector.zeros(1)
shared_data.actuator.torqueEnable = vector.zeros(1)
shared_data.actuator.torqueEnableChanged = vector.zeros(1)
shared_data.actuator.hardnessChanged = vector.zeros(1)
shared_data.actuator.gainChanged = vector.zeros(1)
shared_data.actuator.bias = vector.zeros(nJoint)
shared_data.actuator.hardness = vector.ones(nJoint)
shared_data.sensor.updatedCount  = vector.zeros(1)


--Actuator variables
shared_data.actuator.command_position = vector.zeros(nJoint) --target position
shared_data.actuator.command_position2 = vector.zeros(nJoint) --current target position (with vel limit)

shared_data.actuator.torque_enable = vector.zeros(nJoint) --joint specific torque enable values
shared_data.actuator.torque_enable_changed = vector.zeros(1)

shared_data.actuator.command_current = vector.zeros(nJoint)
shared_data.actuator.command_velocity = vector.zeros(nJoint)
shared_data.actuator.command_torque = vector.zeros(nJoint) --for webots only for now



------------------------
-- Call the initializer
memory.init_shm_segment(..., shared_data, shared_data_sz)
