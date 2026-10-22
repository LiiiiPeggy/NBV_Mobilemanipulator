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
    # Default: FKIE freespace ROI around (-5.36,-5.685) from nbv_planner_*.yaml
    # Override with ~boundary_xy: [[x,y], ...] if needed.
    default_xy = [
        (-8.3603, -2.6852),
        (-2.3603, -2.6852),
        (-2.3603, -8.6852),
        (-8.3603, -8.6852),
    ]
    boundary_xy = rospy.get_param("~boundary_xy", default_xy)
    for x, y in boundary_xy:
        poly.polygon.points.append(Point32(x=float(x), y=float(y), z=0.0))

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
    if result is None:
        rospy.logerr("Action finished with no result (state=%s, often LOST/preempted)", state)
        sys.exit(5)
    n_goals = len(result.goals) if result.goals else 0
    rospy.loginfo(
        "result: complete_exploration=%s request_base_pose=%s goals=%d explored_boundary_id=%s",
        result.complete_exploration,
        result.request_base_pose,
        n_goals,
        result.explored_boundary_id,
    )
    if result.goal_pose_3d and result.goal_pose_3d.header.frame_id:
        p = result.goal_pose_3d.pose.position
        rospy.loginfo(
            "goal_pose_3d frame=%s xyz=(%.3f, %.3f, %.3f)",
            result.goal_pose_3d.header.frame_id,
            p.x,
            p.y,
            p.z,
        )
    for i, g in enumerate(result.goals or []):
        p = g.pose.position
        print(
            "GOAL[%d] frame=%s xyz=(%.3f, %.3f, %.3f)"
            % (i, g.header.frame_id, p.x, p.y, p.z)
        )
    # State 3 == SUCCEEDED in actionlib
    if state != 3:
        sys.exit(4)
    print("VERIFY_FKIE_ACTION_OK")
    if n_goals > 0:
        print("VERIFY_FKIE_NONEMPTY_GOALS")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSInterruptException:
        sys.exit(1)
