local memory = require'memory'
local vector = require'vector'


--Human input memory

-- shared properties
local shared = {};
local shsize = {};


shared.motionplan={}
shared.motionplan.targetxya=vector.zeros(3)
shared.motionplan.targettype=vector.zeros(1)
shared.motionplan.targetsize=vector.zeros(1)
shared.motionplan.targetid=vector.zeros(1)
shared.motionplan.graspwidth=vector.zeros(1) --paper can plastic glass plasticbag
shared.motionplan.grasptype=vector.zeros(1) --
shared.motionplan.verticalgrab=vector.zeros(1) --for bags and fallen down bottles
shared.motionplan.execute=vector.zeros(1) --for bags and fallen down bottles
shared.motionplan.gripexecute=vector.zeros(1) --for bags and fallen down bottles
shared.motionplan.pickuptype=vector.zeros(1) --0 for side, 1 for direct vertical, 2 for offsetted vertical (for snack)

shared.motionplan.autorun=vector.zeros(1)
shared.motionplan.show_arrow=vector.zeros(1)

shared.voice={}
shared.voice.execute=vector.zeros(1)
shared.voice.str="defaultstrxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

-- Call the initializer
memory.init_shm_segment(..., shared, shsize)
