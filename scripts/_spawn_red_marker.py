#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel, GetModelState, DeleteModel
from geometry_msgs.msg import Pose, Point, Quaternion
rospy.init_node("big_marker", anonymous=True)
spawn=rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
get=rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
delete=rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
name="probe_big_red_box"
try:
  if get(name,"world").success:
    delete(name); rospy.sleep(0.2)
except Exception:
  pass
sdf="""<?xml version="1.0"?>
<sdf version="1.6"><model name="probe_big_red_box"><static>true</static>
<link name="link"><visual name="v"><pose>0 0 0 0 0 0</pose><geometry><box><size>0.25 0.25 0.25</size></box></geometry>
<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse><emissive>0.3 0 0 1</emissive></material></visual>
<collision name="c"><geometry><box><size>0.25 0.25 0.25</size></box></geometry></collision>
</link></model></sdf>"""
pose=Pose(position=Point(-2.45,-6.35,0.85), orientation=Quaternion(0,0,0,1))
r=spawn(name,sdf,"",pose,"world")
print("big_marker", r.success, r.status_message, "exists", get(name,"world").success)
