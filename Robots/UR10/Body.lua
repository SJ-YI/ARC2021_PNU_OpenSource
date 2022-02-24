--------------------------------
-- Body abstraction for THOR-OP and Naver N1Bot
-- (c) 2013,2014 Stephen McGill, Seung-Joon Yi
-- Cleaned up for non-humanoid robot
--------------------------------
-- Utilities
local vector = require'vector'
local util = require'util'
local si = require'simple_ipc'
local Kinematics = require'ARM6Kinematics'
local mpack  = require'msgpack.MessagePack'.pack
require'dcm'

local Body = {}
local dcm_ch = si.new_publisher'dcm!'
local get_time = require'unix'.time
local usleep = require'unix'.usleep
local vslice = require'vector'.slice



-- Body sensors

for sensor, n_el in pairs(dcm.sensorKeys) do
  local ptr, ptr_t
  if dcm.sensorPtr then ptr = dcm.sensorPtr[sensor] end
  local function get(idx1, idx2, needs_wait)
    if IS_WEBOTS then needs_wait=false end     --for some reason, webot makes this code wait, slowing down simulation a lot
		local start, stop = idx1 or 1, idx2 or n_el
	  return vslice(ptr, start-1, stop-1)   	-- For cdata, use -1
	end
  Body['get_'..sensor] = get -- Anthropomorphic access to dcm
  for part, jlist in pairs(Config.parts) do
	  local not_synced = sensor~='position'
    local idx1, idx2 = jlist[1], jlist[#jlist]
    Body['get_'..part:lower()..'_'..sensor] = function(idx)
			return get(idx1, idx2, not_synced)
    end -- Get
  end -- End anthropomorphic
end




-- Body actuators
for actuator, n_el in pairs(dcm.actuatorKeys) do
	-- Only command_position is constantly synced
	-- Other commands need to be specially sent to the Body
  -- TODO: Check the torque usage in NX motors...
	local not_synced = not (actuator=='command_position' or actuator=='command_torque')
  local ptr = dcm.actuatorPtr and dcm.actuatorPtr[actuator]
  local idx
	local function set(val, idx1, idx2)
		local changed_ids = {}
		if idx2 then
			if type(val)=='number' then
				for idx=idx1, idx2 do
					changed_ids[idx] = true
					ptr[idx - 1] = val 		-- cdata is -1
				end
			else
				for i,v in ipairs(val) do
					idx = idx1 + i - 1
					changed_ids[idx] = true
					if idx>idx2 then break else ptr[idx - 1] = v end
				end
			end
		elseif idx1 then
			if type(val)=='number' then
				changed_ids[idx1] = true
				ptr[idx1 - 1] = val
			else
				for i, v in ipairs(val) do
					idx = idx1 + i - 1
					changed_ids[idx] = true
					ptr[idx - 1] = v
				end
			end
		else -- No index means set all actuators... Uncommon
			if type(val)=='number' then
				for i=0, n_el-1 do
					changed_ids[i + 1] = true
					ptr[i] = val
				end
			else
				for i, v in ipairs(val) do
					changed_ids[i] = true
					ptr[i - 1] = v
				end
			end
		end
		-- Send msg to the dcm, just string of the id
		if not_synced then dcm_ch:send(mpack({wr_reg=actuator, ids=changed_ids})) end
	end
	local function get(idx1, idx2, needs_wait)
		idx1 = idx1 or 1
		idx2 = idx2 or n_el
    if needs_wait then
			local ids = {}
			for id = idx1, idx2 do ids[id] = true end
			dcm_ch:send(mpack({rd_reg=actuator, ids=ids}))
			unix.usleep(1e4) -- 100Hz assumed for the wait period to be in SHM
    end
		return vslice(ptr, idx1 - 1, idx2 - 1) -- For cdata, use -1
	end
	-- Export
  Body['set_'..actuator] = set
  Body['get_'..actuator] = get
  --------------------------------
  -- Anthropomorphic access to dcm
  -- TODO: Do not use string concatenation to call the get/set methods of Body
  for part, jlist in pairs(Config.parts) do
		local idx1, idx2, idx = jlist[1], jlist[#jlist], nil
		Body['get_'..part:lower()..'_'..actuator] = function(idx)
			local needs_wait = not (actuator=='command_position')
			if idx then return get(jlist[idx], needs_wait) else return get(idx1, idx2, needs_wait) end
		end
		Body['set_'..part:lower()..'_'..actuator] = function(val, i)
			if i then return set(val, jlist[i])
      else return set(val, idx1, idx2)
      end
		end
  end 	-- End anthropomorphic
end


--dummy functions
function Body.set_position_p() end
-----------------------------------------



function Body.entry() end
function Body.update() end
function Body.exit() end

function Body.enable_read(chain) dcm_ch:send(mpack({bus=chain,key='enable_read', val=true})) end
function Body.disable_read(chain) dcm_ch:send(mpack({bus=chain,key='enable_read', val=false})) end

if IS_WEBOTS then -- Webots compatibility
	local WebotsBody
  webots = require'webots' --now a global variable
  Body.enable_read = function(chain) end
  Body.disable_read = function(chain) end
  Body.exit = function() end
  webots.wb_robot_init()
  Body.timeStep = webots.wb_robot_get_basic_time_step()
  WebotsBody = require'WebotsBody'
  last_webots_time=webots.wb_robot_get_time()
	function Body.entry()	WebotsBody.entry(Body)end
  function Body.update() WebotsBody.update(Body) end
  get_time = webots.wb_robot_get_time
end

--hack for waist-less robot
function Body.get_waist_position() return{0,0} end

-- Exports for use in other functions
Body.get_time = get_time
Body.nJoint = nJoint
Body.jointNames = jointNames
Body.parts = Config.parts
Body.Kinematics = Kinematics

return Body
