assert(Config, 'Need a pre-existing Config table!')
local vector = require'vector'

Config.start_wizards={
    -- {'check', },
    -- {'xb360', },
    -- {'dynamixel',},
  }

Config.sensors={
    pose=true,
    -- rosio="UR5ROS",
    kinect = "nil_wizard",
}


if IS_WEBOTS then
  Config.kinect={
    RGBname="kinect2RGB",
    Dname="kinect2D",
    -- relp={0.14,0,-0.035,   180*DEG_TO_RAD,0,0}, --from final wrist joint
    relp={0.17,0,-0.03,   180*DEG_TO_RAD,0,0}, --from final wrist joint
    cx=320.5,
    cy=240.5,
    fov = 1.0472, --60 degree horizontal
    fxy=554.2546911--for 60 degree fov
  }
else
  Config.kinect={
    RGBname="kinect2RGB",
    Dname="kinect2D",
    relp={0.17,0.025,0.028,  180*DEG_TO_RAD,0,0}, --from final wrist joint
    cx=321.9649, --camera value
    cy=252.1346, --camera value
    fov = 1.0472, --this is NOT correct...but is not used anyway
    fxy=624.5385742 --correct camera value
  }
end

--HACK FOR WEBOTS TESTING (realsense model)--
-- -----------------------------------
Config.kinect.relp={0.177,0,-0.03,   180*DEG_TO_RAD,0,0} --from final wrist joint
Config.kinect.cx=320.5
Config.kinect.cy=240.5
Config.kinect.fov = 1.0472 --60 degree horizontal
Config.kinect.fxy=462.11819--for 60 degree fov
-- --------------------------------------

--fxy = width/2.0 / tan(fov/2.0)
-- 2*atan(width/2/fxy) = fov
-- Config.camera_timestep = 1000 --500ms
Config.kinect_timestep = 20


return Config
