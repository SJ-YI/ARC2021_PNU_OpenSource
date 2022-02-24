local memory = require'memory'
local vector = require'vector'

-- shared properties
local shared = {};
local shsize = {};


shared.game = {};
shared.game.state = vector.zeros(1)

shared.game.selfdestruct = vector.zeros(1)


shared.processes={}
--current state / count / fps
shared.processes.dynamixel = vector.zeros(3)
shared.processes.xb360 = vector.zeros(3)

-- shared.processes.velodyne2 = vector.zeros(3)
-- shared.processes.velodyne3 = vector.zeros(3)
-- shared.processes.obstacle = vector.zeros(3)
-- shared.processes.slam3d = vector.zeros(3)
-- shared.processes.bldc = vector.zeros(3)
-- shared.processes.state = vector.zeros(3)

-- shared.processes.rplidar = vector.zeros(2)
-- shared.processes.imu = vector.zeros(2)
-- shared.processes.thetas = vector.zeros(2)


-- Keep track of every state machine
-- Use the Config'd FSMs
shared.fsm = {}
if Config and Config.fsm then
  for sm, en in pairs(Config.fsm.enabled) do
    shared.fsm[sm] = ''
  end
end

-- Call the initializer
memory.init_shm_segment(..., shared, shsize)
