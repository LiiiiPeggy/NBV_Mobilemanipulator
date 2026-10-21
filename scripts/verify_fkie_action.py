#!/usr/bin/env python3
"""Send a NbvPlanner action goal and print the result.

Action server: /robot/nbv_rrt (see NBVPlanner.cpp).
Requires: roscore, nbvplanner_node, camera_pose, and optionally octomap.
"""
from __future__ import print_function

import sys

import actionlib
import rospy
from geometry_msgs.msg import Point32, Polygon, PolygonStamped, PoseStamped
from std_msgs.msg import Header

from fkie_measurement_msgs.msg import BoundaryPolygon, MeasurementEstimation
from fkie_nbv_planner.msg import NbvPlannerAction, NbvPlannerGoal


def main():
    rospy.init_node("verify_fkie_action")
    server = rospy.get_param("~action_server", "/robot/nbv_rrt")
    timeout = rospy.get_param("~timeout", 60.0)

    client = actionlib.SimpleActionClient(server, NbvPlannerAction)
    rospy.loginfo("Waiting for action server: %s", server)
    if not client.wait_for_server(rospy.Duration(timeout)):
        rospy.logerr("Action server not available: %s", server)
        sys.exit(2)

    goal = NbvPlannerGoal()
    goal.header = Header(stamp=rospy.Time.now(), frame_id="world")
    goal.arm_in_prepacked = True

    poly = PolygonStamped()
    poly.header = goal.header
    # 10m x 10m square exploration boundary around origin
    for x, y in [(-5.0, -5.0), (5.0, -5.0), (5.0, 5.0), (-5.0, 5.0)]:
        poly.polygon.points.append(Point32(x=x, y=y, z=0.0))

    boundary = BoundaryPolygon()
    boundary.id = 1
    boundary.source_type = "verify"
    boundary.min_height = 0.0
    boundary.max_height = 2.0
    boundary.polygon = poly
    goal.current_boundary = boundary

    est = MeasurementEstimation()
    est.header = goal.header
    est.grid_size = 1.0
    goal.estimations = est

    rospy.loginfo("Sending NbvPlanner goal...")
    client.send_goal(goal)
    finished = client.wait_for_result(rospy.Duration(timeout))
    if not finished:
        rospy.logerr("Action timed out")
        client.cancel_goal()
        sys.exit(3)

    state = client.get_state()
    result = client.get_result()
    rospy.loginfo("Action state=%s", state)
    rospy.loginfo(
        "result: complete_exploration=%s request_base_pose=%s goals=%d explored_boundary_id=%s",
        result.complete_exploration,
        result.request_base_pose,
        len(result.goals) if result.goals else 0,
        result.explored_boundary_id,
    )
    # State 3 == SUCCEEDED in actionlib
    if state != 3:
        sys.exit(4)
    print("VERIFY_FKIE_ACTION_OK")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(1)
