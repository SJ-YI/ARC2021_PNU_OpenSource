#!/usr/bin/env python

from controller import Supervisor
from multiprocessing import Process
import time
import random
from math import pi as PI
from pyquaternion import Quaternion   # (ref) http://kieranwynn.github.io/pyquaternion/

timestep = 30

def coca_bottle(x,y,z,ang):
  supervisor = Supervisor()
  robot_node = supervisor.getFromDef("coca_bottle")
  trans_field = robot_node.getField("translation")
  rotation_field = robot_node.getField("rotation")

  ROTATION = [k for k in Quaternion(axis=[0 , 1, 0], angle=ang)]

  POSITION = [x, y, z]
  trans_field.setSFVec3f(POSITION)
  rotation_field.setSFRotation([0, 1, 0, ang])
  robot_node.resetPhysics()
  while(True):
    supervisor.step(timestep)

def cocacan(x,y,z, ang):
  supervisor = Supervisor()
  robot_node = supervisor.getFromDef("cocacan")
  trans_field = robot_node.getField("translation")
  rotation_field = robot_node.getField("rotation")
  ROTATION = [k for k in Quaternion(axis=[0 , 1, 0], angle=ang)]

  POSITION = [x, y, z]
  trans_field.setSFVec3f(POSITION)
  rotation_field.setSFRotation([0, 1, 0, ang])
  robot_node.resetPhysics()
  while(True):
    supervisor.step(timestep)

def milk2l(x,y,z, ang):
  supervisor = Supervisor()
  robot_node = supervisor.getFromDef("milk2l")
  trans_field = robot_node.getField("translation")
  rotation_field = robot_node.getField("rotation")
  ROTATION = [k for k in Quaternion(axis=[0 , 1, 0], angle=ang)]

  POSITION = [x, y, z]
  trans_field.setSFVec3f(POSITION)
  rotation_field.setSFRotation([0, 1, 0, ang])
  robot_node.resetPhysics()
  while(True):
    supervisor.step(timestep)

def milk250ml(x,y,z, ang):
  supervisor = Supervisor()
  robot_node = supervisor.getFromDef("milk250ml")
  trans_field = robot_node.getField("translation")
  rotation_field = robot_node.getField("rotation")
  ROTATION = [k for k in Quaternion(axis=[0 , 1, 0], angle=ang)]

  POSITION = [x, y, z]
  trans_field.setSFVec3f(POSITION)
  rotation_field.setSFRotation([0, 1, 0, ang])
  robot_node.resetPhysics()
  while(True):
    supervisor.step(timestep)

def notebook(x,y,z, ang):
  supervisor = Supervisor()
  robot_node = supervisor.getFromDef("notebook")
  trans_field = robot_node.getField("translation")
  rotation_field = robot_node.getField("rotation")
  ROTATION = [k for k in Quaternion(axis=[0 , 1, 0], angle=ang)]

  POSITION = [x, y, z]
  trans_field.setSFVec3f(POSITION)
  rotation_field.setSFRotation([0, 1, 0, ang])
  robot_node.resetPhysics()
  while(True):
    supervisor.step(timestep)

def pain_killer(x,y,z, ang):
  supervisor = Supervisor()
  robot_node = supervisor.getFromDef("pain_killer")
  trans_field = robot_node.getField("translation")
  rotation_field = robot_node.getField("rotation")
  ROTATION = [k for k in Quaternion(axis=[0 , 1, 0], angle=ang)]

  POSITION = [x, y, z]
  trans_field.setSFVec3f(POSITION)
  rotation_field.setSFRotation([0, 1, 0, ang])
  robot_node.resetPhysics()
  while(True):
    supervisor.step(timestep)

def plasticbag(x,y,z, ang):
  supervisor = Supervisor()
  robot_node = supervisor.getFromDef("plasticbag")
  trans_field = robot_node.getField("translation")
  rotation_field = robot_node.getField("rotation")
  ROTATION = [k for k in Quaternion(axis=[0 , 1, 0], angle=ang)]

  POSITION = [x, y, z]
  trans_field.setSFVec3f(POSITION)
  rotation_field.setSFRotation([0, 1, 0, ang])
  robot_node.resetPhysics()
  while(True):
    supervisor.step(timestep)

def tissue(x,y,z, ang):
  supervisor = Supervisor()
  robot_node = supervisor.getFromDef("tissue")
  trans_field = robot_node.getField("translation")
  rotation_field = robot_node.getField("rotation")
  ROTATION = [k for k in Quaternion(axis=[0 , 1, 0], angle=ang)]

  POSITION = [x, y, z]
  trans_field.setSFVec3f(POSITION)
  rotation_field.setSFRotation([0, 1, 0, ang])
  robot_node.resetPhysics()
  while(True):
    supervisor.step(timestep)

def waterbottle(x,y,z, ang):
  supervisor = Supervisor()
  robot_node = supervisor.getFromDef("waterbottle")
  trans_field = robot_node.getField("translation")
  rotation_field = robot_node.getField("rotation")
  ROTATION = [k for k in Quaternion(axis=[0 , 1, 0], angle=ang)]

  POSITION = [x, y, z]
  trans_field.setSFVec3f(POSITION)
  rotation_field.setSFRotation([0, 1, 0, ang])
  robot_node.resetPhysics()
  while(True):
    supervisor.step(timestep)

def wine(x,y,z, ang):
  supervisor = Supervisor()
  robot_node = supervisor.getFromDef("wine")
  trans_field = robot_node.getField("translation")
  rotation_field = robot_node.getField("rotation")
  ROTATION = [k for k in Quaternion(axis=[0 , 1, 0], angle=ang)]

  POSITION = [x, y, z]
  trans_field.setSFVec3f(POSITION)
  rotation_field.setSFRotation([0, 1, 0, ang])
  robot_node.resetPhysics()
  while(True):
    supervisor.step(timestep)

def beerbottle(x,y,z, ang):
  supervisor = Supervisor()
  robot_node = supervisor.getFromDef("beerbottle")
  trans_field = robot_node.getField("translation")
  rotation_field = robot_node.getField("rotation")
  ROTATION = [k for k in Quaternion(axis=[0 , 1, 0], angle=ang)]

  POSITION = [x, y, z]
  trans_field.setSFVec3f(POSITION)
  rotation_field.setSFRotation([0, 1, 0, ang])
  robot_node.resetPhysics()
  while(True):
    supervisor.step(timestep)

def beercan(x,y,z, ang):
  supervisor = Supervisor()
  robot_node = supervisor.getFromDef("beercan")
  trans_field = robot_node.getField("translation")
  rotation_field = robot_node.getField("rotation")
  ROTATION = [k for k in Quaternion(axis=[0 , 1, 0], angle=ang)]

  POSITION = [x, y, z]
  trans_field.setSFVec3f(POSITION)
  rotation_field.setSFRotation([0, 1, 0, ang])
  robot_node.resetPhysics()
  while(True):
    supervisor.step(timestep)

def coffecan(x,y,z, ang):
  supervisor = Supervisor()
  robot_node = supervisor.getFromDef("coffecan")
  trans_field = robot_node.getField("translation")
  rotation_field = robot_node.getField("rotation")
  ROTATION = [k for k in Quaternion(axis=[0 , 1, 0], angle=ang)]
  POSITION = [x, y, z]
  trans_field.setSFVec3f(POSITION)
  rotation_field.setSFRotation([0, 1, 0, ang])
  robot_node.resetPhysics()
  while(True):
    supervisor.step(timestep)

if __name__=='__main__':
  y_position = 0.79
  list = [[0.3,y_position,0.5],[0.3,y_position,0.375],[0.3,y_position,0.25],[0.3,y_position,0.125],[0.3,y_position,0],[0.3,y_position,-0.125],[0.3,y_position,-0.25],[0.3,y_position,-0.375],[0.3,y_position,-0.5],
  [0.435,y_position,0.5],[0.435,y_position,0.375],[0.435,y_position,0.25],[0.435,y_position,0.125],[0.435,y_position,0],[0.435,y_position,-0.125],[0.435,y_position,-0.25],[0.435,y_position,-0.375],[0.435,y_position,-0.5],
  [0.57,y_position,0.5],[0.57,y_position,0.375],[0.57,y_position,0.25],[0.57,y_position,0.125],[0.57,y_position,0],[0.57,y_position,-0.125],[0.57,y_position,-0.25],[0.57,y_position,-0.375],[0.57,y_position,-0.5],
  [0.705,y_position,0.5],[0.705,y_position,0.375],[0.705,y_position,0.25],[0.705,y_position,0.125],[0.705,y_position,0],[0.705,y_position,-0.125],[0.705,y_position,-0.25],[0.705,y_position,-0.375],[0.705,y_position,-0.5],
  [0.84,y_position,0.5],[0.84,y_position,0.375],[0.84,y_position,0.25],[0.84,y_position,0.125],[0.84,y_position,0],[0.84,y_position,-0.125],[0.84,y_position,-0.25],[0.84,y_position,-0.375],[0.84,y_position,-0.5]]

  random.shuffle(list)

  #random_angle = random.uniform(0, PI*2) - PI


  th1  = Process(target=coca_bottle,  args=(list[0][0],  list[0][1],  list[0][2],  random.uniform(0, PI*2) - PI))
  th2  = Process(target=cocacan,      args=(list[1][0],  list[1][1],  list[1][2],  random.uniform(0, PI*2) - PI))
  th3  = Process(target=milk2l,       args=(list[2][0],  list[2][1],  list[2][2],  random.uniform(0, PI*2) - PI))
  th4  = Process(target=milk250ml,    args=(list[3][0],  list[3][1],  list[3][2],  random.uniform(0, PI*2) - PI))
#  th5  = Process(target=notebook,    args=(list[4][0],  list[4][1],  list[4][2],  random.uniform(0, PI*2) - PI))
  th6  = Process(target=pain_killer,  args=(list[5][0],  list[5][1],  list[5][2],  random.uniform(0, PI*2) - PI))
  th7  = Process(target=plasticbag,   args=(list[6][0],  list[6][1],  list[6][2],  random.uniform(0, PI*2) - PI))
  th8  = Process(target=tissue,       args=(list[7][0],  list[7][1],  list[7][2],  random.uniform(0, PI*2) - PI))
  th9  = Process(target=waterbottle,  args=(list[8][0],  list[8][1],  list[8][2],  random.uniform(0, PI*2) - PI))
  th10 = Process(target=wine,         args=(list[9][0],  list[9][1],  list[9][2],  random.uniform(0, PI*2) - PI))
  th11 = Process(target=beerbottle,   args=(list[10][0], list[10][1], list[10][2], random.uniform(0, PI*2) - PI))
  th12 = Process(target=beercan,      args=(list[11][0], list[11][1], list[11][2], random.uniform(0, PI*2) - PI))
  th13 = Process(target=coffecan,     args=(list[12][0], list[12][1], list[12][2], random.uniform(0, PI*2) - PI))
  th1.start()
  time.sleep(0.01)
  th2.start()
  time.sleep(0.01)
  th3.start()
  time.sleep(0.01)
  th4.start()
  time.sleep(0.01)
#  th5.start()
#  time.sleep(0.01)
  th6.start()
  time.sleep(0.01)
  th7.start()
  time.sleep(0.01)
  th8.start()
  time.sleep(0.01)
  th9.start()
  time.sleep(0.01)
  th10.start()
  time.sleep(0.01)
  th11.start()
  time.sleep(0.01)
  th12.start()
  time.sleep(0.01)
  th13.start()
  time.sleep(0.01)
