--------------------------------
-- World Communication Module
-- (c) 2013 Stephen McGill
--------------------------------
local vector = require'vector'
local memory = require'memory'

-- shared properties
local shared = {}
local shsize = {}

shared.robot = {}
shared.robot.pose = vector.zeros(3)
shared.robot.pose0 = vector.zeros(3)
shared.robot.pose_odom = vector.zeros(3)
shared.robot.pose_gps = vector.zeros(3)
shared.robot.rostime = vector.zeros(1)
shared.robot.webots = vector.zeros(1)
shared.robot.totaldist = vector.zeros(1)

MAX_OBJECTS_NO=255

--for cube pickup task
shared.objects={}
shared.objects.num=vector.zeros(1)
shared.objects.xpos=vector.zeros(MAX_OBJECTS_NO)
shared.objects.ypos=vector.zeros(MAX_OBJECTS_NO)
shared.objects.zpos=vector.zeros(MAX_OBJECTS_NO)
shared.objects.yaw=vector.zeros(MAX_OBJECTS_NO)
shared.objects.grabyaw=vector.zeros(MAX_OBJECTS_NO)
shared.objects.grabtype=vector.zeros(MAX_OBJECTS_NO)
shared.objects.min_width=vector.zeros(MAX_OBJECTS_NO)
shared.objects.max_width=vector.zeros(MAX_OBJECTS_NO)
shared.objects.id=vector.zeros(MAX_OBJECTS_NO)
shared.objects.selected=vector.zeros(1)
shared.objects.enable=vector.zeros(1)



-- Call the initializer
memory.init_shm_segment(..., shared, shsize)
