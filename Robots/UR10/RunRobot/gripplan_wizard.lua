#!/usr/bin/env luajit
local pwd = os.getenv'PWD'
if string.find(pwd,"Run") then  dofile('../include.lua')
else    dofile('./include.lua') end

require'wcm'
require'hcm'
local unix=require'unix'
local ffi = require'ffi'
local vector=require'vector'
local util=require'util'
local T = require'Transform'

hcm.set_motionplan_gripexecute(0)

local classnames={
  {"coke", "Coke Can",3},
  {"beer", "Beer Can",3},
  {"coffee", "Coffee",2}, --Coffee is now paper
  {"milk","Small Milk",2},
  {"wine","Wine",5},
  {"water","Water Bottle",1},
  {"coke_big","Coke Bottle",1},--NOT a 2 liter bottle, though
  {"milk_big","Big Milk",2},
  {"beer_big","Beer Bottle",5},
  {"snack","Snack",4},
  {"paper","Toilet Paper",2},
  {"painkiller","Painkiller",1},
}

local typenames={"Plastic","Paper","Can","Bag","Glass"}

local function update_grip()
  print("===================================================================================")
  local objnum=wcm.get_objects_num()
  local oxp,oyp,ozp,oyaw,oid,ominw, omaxw=
    wcm.get_objects_xpos(),wcm.get_objects_ypos(),wcm.get_objects_zpos(),
    wcm.get_objects_yaw(),wcm.get_objects_id(),
    wcm.get_objects_min_width(),wcm.get_objects_max_width()
  local ograbyaw=wcm.get_objects_grabyaw()
  local ograbtype=wcm.get_objects_grabtype() --0 for side, 1 for direct vertical, 2 for offset vertical (bag)

  local yaws={180*DEG_TO_RAD, 135*DEG_TO_RAD, -135*DEG_TO_RAD, 90*DEG_TO_RAD, -90*DEG_TO_RAD, 0}


  local yaws={180*DEG_TO_RAD,
  165*DEG_TO_RAD,-165*DEG_TO_RAD,
  150*DEG_TO_RAD,-150*DEG_TO_RAD,
  135*DEG_TO_RAD,-135*DEG_TO_RAD,
  120*DEG_TO_RAD,-120*DEG_TO_RAD,
  105*DEG_TO_RAD,-105*DEG_TO_RAD,
   90*DEG_TO_RAD, -90*DEG_TO_RAD, 0}



  local max_dist_wrist=0.9
  local wrist_length = 0.30
  local wrist_length = 0.40

  local min_dist_wrist=0.25
  local min_dist_wrist=0.40

  local wrist_x_check={-0.60,0.08} --for 8cm side move

  local wrist_x_check={-0.55,0.04} --for no side move
  local dist_th = 0.09

  for i=1,objnum do
    local x,y=oxp[i],oyp[i]
    local good_yaw=nil

    local objname="unknown"
    if oid[i]>0 then objname=classnames[oid[i]][2] end

    print(string.format("====Checking Object %d(%s)  (%.2f %.2f) ====================", i ,objname,x,y) )


    if ozp[i]<0.10 and omaxw[i]>ozp[i] or oid[i]==10 then
      --VERTICAL GRAB!!!
      ograbtype[i]=1
      ograbyaw[i]=oyaw[i]+90*DEG_TO_RAD
      if ograbyaw[i]>180*DEG_TO_RAD then ograbyaw[i]=ograbyaw[i]-360*DEG_TO_RAD end
    else
      for j=1,#yaws do
        if not good_yaw then
          local pose_grab={x,y,yaws[j]}
          local min_dist=999
          for ii=1,objnum do
            -- if ii~=i then
            if ii~=i and oid[ii]~=10 then --ignore snack
              local pose_abs={oxp[ii],oyp[ii],0}
              local rel_pose=util.pose_relative(pose_abs, pose_grab)
              if rel_pose[1]>wrist_x_check[1] and rel_pose[1]<wrist_x_check[2] then
                local y_dist=math.abs(rel_pose[2])-math.min(omaxw[ii]/2, 0.05)
                if y_dist<0.20 and y_dist<min_dist then min_dist=y_dist end
              end
            end
          end
          local pose_wrist=util.pose_global({-wrist_length,0,0},pose_grab)
          local dist_wrist=math.sqrt( pose_wrist[1]*pose_wrist[1]+pose_wrist[2]*pose_wrist[2] )
          if dist_wrist>=max_dist_wrist or dist_wrist<min_dist_wrist then
            print(string.format("yaw %d: wrist dist %.2f min_dist %.2f WRIST DIST ERR",yaws[j]/DEG_TO_RAD,dist_wrist, min_dist))
          elseif min_dist<dist_th then
            print(string.format("yaw %d: wrist dist %.2f min_dist %.2f MIN DIST ERR",yaws[j]/DEG_TO_RAD,dist_wrist, min_dist))
          else
            print(string.format("yaw %d: wrist dist %.2f min_dist %.2f",yaws[j]/DEG_TO_RAD,dist_wrist, min_dist))
          end
          if min_dist>dist_th and dist_wrist<max_dist_wrist and dist_wrist>min_dist_wrist then good_yaw=yaws[j] end
        end --end good_yaw
      end --end j
      if good_yaw then
        ograbyaw[i]=good_yaw
        print(string.format("====Object %d(%s): good yaw %.1f====", i ,objname, good_yaw/DEG_TO_RAD))
      else
        ograbyaw[i]=-999
        print(string.format("====Object %d(%s): no grab direction====", i,objname))
      end
      ograbtype[i]=0
    end

  end --end objnum
  wcm.set_objects_grabyaw(ograbyaw)
  wcm.set_objects_grabtype(ograbtype)

  print("===================================================================================")

  local best_item=0
  local best_dist=999
  local camera_x=1.35

  for i=1,objnum do
    local dist= (camera_x-oxp[i])*(camera_x-oxp[i]) + oyp[i]*oyp[i]
    if ograbyaw[i]>-999 and dist<best_dist and oid[i]~=10 and oid[i]>0 then --don't choose snack or unknown first
      best_item, best_dist=i,dist
    end
  end

  if best_item==0 then
    for i=1,objnum do
      if oid[i]==10 then best_item=i end
    end
  end








  if best_item>0 then
    local targetid,targettype,targetname=oid[best_item],hcm.get_motionplan_targettype(),"unknown"
    if targetid==0 then
      targettype=targettype+1
      if targettype>5 then targettype=1 end
    else
      targettype=classnames[targetid][3]
      targetname=classnames[targetid][2]
    end

    hcm.set_voice_str(string.format("Moving %s to %s",targetname,typenames[targettype] ))
    hcm.set_voice_execute(1)

    if omaxw[best_item]>0.10 then hcm.set_motionplan_graspwidth(0.8) --semi grab
    else hcm.set_motionplan_graspwidth(1.1)  --full grab
    end


    hcm.set_motionplan_targettype(targettype)
    wcm.set_objects_selected(best_item)
    hcm.set_motionplan_targetxya({oxp[best_item], oyp[best_item], ograbyaw[best_item] })
    hcm.set_motionplan_targetsize(omaxw[best_item])
    hcm.set_motionplan_grasptype(ograbtype[best_item])
    hcm.set_motionplan_targetid(targetid)



    if Config.run_auto then	hcm.set_motionplan_execute(1) end

  else
    wcm.set_objects_selected(0)
    hcm.set_motionplan_execute(0)
  end
end


local running=true
local t_last_update=unix.time()
local t_last_debug=unix.time()


while running do
  local gripexecute=hcm.get_motionplan_gripexecute()
  if gripexecute>0 then
    hcm.set_motionplan_gripexecute(0)
    update_grip()
  end
  local t=unix.time()
  if t-t_last_debug>5 then
    local pose=wcm.get_robot_pose()
    print(string.format("Gripplan: waiting"  ))
    t_last_debug=t
  end
  unix.usleep(1E6*0.01)
end
