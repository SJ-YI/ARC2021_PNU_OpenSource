------------------------------
--ROBOT NAMES
------------------------------
IS_LOCALHOST = false

-- Global Config
Config = {PLATFORM_NAME = 'UR10',demo = false,}
local exo = {'FSM','Robot','Perception'} --for cube picking task
-- local exo = {'FSM_Teleop','Robot_HYgripper','Perception'} --for VR teleop task


-- yolo_path="arc_0929"
-- Config.use_back_mask=false

-- yolo_path="arc_1011"
-- Config.use_back_mask=true


--yolo_path="arc_1013"
yolo_path="arc_1019"
Config.use_back_mask=false

Config.use_yolo=true

-- Config.auto_save_logs=true
-- Config.auto_save_logs=false
-- Config.run_auto = true





if Config.use_yolo then
  Config.robot_startup={
    start_wizards={
      {'perceptionyolo', },
  		{'rosio', },
  		{'gripplan', },
      {'voice', },
    },
    start_processes={
       "roslaunch my_launch webots_ur10_arc_yolo.launch yolo_data_path:="..yolo_path, --launch rviz, yolo, surfacedetect, point cloud cloudifier
       "luajit test_rosio.lua", --launch MB1 rviz
  	},
  }


else


  Config.robot_startup={
    start_wizards={
      {'perception', },
  		{'rosio', },
    },
    start_processes={
       "roslaunch my_launch webots_ur10_arc.launch", --launch MB1 rviz
       "luajit test_rosio.lua", --launch MB1 rviz
  	},
  }


end















-- --Required lua codes and other processes for webots simulation
Config.webots_startup={
	world_name="UR5E_Zenga",--webots world name
  test_file="test_move",--keyboard I/O file for webots control
	start_wizards={
    -- {'fake_perception', },
    {'perception', },
    {'rospub', },
	},
  start_processes={
    "roslaunch my_launch webots_zeus.launch", --launch MB1 rviz
  }
}



-----------------------------------
-- Load Paths and Configurations --
-----------------------------------
-- Custom Config files
local pname = {HOME, '/Config/',Config.PLATFORM_NAME, '/?.lua;', package.path}
package.path = table.concat(pname)
for _,v in ipairs(exo) do
	--print('Loading', v)
	local fname = {'Config_', Config.PLATFORM_NAME, '_', v}
	local filename = table.concat(fname)
  assert(pcall(require, filename))
end

-- Custom motion libraries
if Config.fsm.libraries then
	for i,sm in pairs(Config.fsm.libraries) do
		local pname = {HOME, '/Player/', i,'/' ,sm, '/?.lua;', package.path}
		package.path = table.concat(pname)
	end
end

-- Finite state machine paths
if Config.fsm.enabled then
	for sm, en in pairs(Config.fsm.enabled) do
		if en then
			local selected = Config.fsm.select and Config.fsm.select[sm]
			if selected then
				local pname = {HOME, '/Player/', sm, 'FSM/', selected, '/?.lua;', package.path}
				package.path = table.concat(pname)
			else --default fsm
				local pname = {HOME, '/Player/', sm, 'FSM/', '?.lua;', package.path}
				package.path = table.concat(pname)
			end
		end
	end
end


-----------------------------------
Config.testfile = 'test_zeus' --for webots

return Config
