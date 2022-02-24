--------------------------------
-- Motion Communication Module
-- (c) 2013 Stephen McGill, Seung-Joon Yi
--------------------------------
local memory = require'memory'
local vector = require'vector'

-- shared properties
local shared = {}
local shsize = {}

--Leg bias and walk params info
shared.leg={}
shared.leg.bias=vector.zeros(12)

shared.walk = {}
shared.walk.vel        = vector.zeros(3)

--Motion states
shared.motion={}
shared.motion.state = vector.zeros(1)


memory.init_shm_segment(..., shared, shsize)
