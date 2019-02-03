import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_interface import Limb

## solving IK
def ik_service_client(limb = "right", use_advanced_options = False, position_xyz=[0,0,0], \
                      orientation_xyzw=[0,0,0,0], seed_position=[0.7, 0.4, -1.7, 1.4, -1.1, -1.6, -0.4]):
    """

    :param limb: "right" default
    :param use_advanced_options: False default
    :param position_xyz: cartesian XYZ [0 0 0] default
    :param orientation_xyzw: quaternion for identifying rotation of the end
    :param seed_position: where to start the IK optimization
    :return:
   """
    # return value class
    class ret:
        def __init__(self):
         self.result=False
         self.angle=[]
    # initialize an instance of return value
    ret=ret()
    
    # call IK service
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    # target pose
    poses = {
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=position_xyz[0],
                    y=position_xyz[1],
                    z=position_xyz[2],
                ),
                orientation=Quaternion(
                    x=orientation_xyzw[0],
                    y=orientation_xyzw[1],
                    z=orientation_xyzw[2],
                    w=orientation_xyzw[3],
                ),
            ),
        ),
    }
    # Add desired pose for inverse kinematics
    ikreq.pose_stamp.append(poses[limb])
    # Request inverse kinematics from base to "right_hand" link
    ikreq.tip_names.append('right_hand')

    if (use_advanced_options):
        # Optional Advanced IK parameters
        rospy.loginfo("Running Advanced IK Service Client example.")
        # The joint seed is where the IK position solver starts its optimization
        ikreq.seed_mode = ikreq.SEED_USER
        seed = JointState()
        seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                     'right_j4', 'right_j5', 'right_j6']
        seed.position = seed_position
        ikreq.seed_angles.append(seed)

        # Once the primary IK task is solved, the solver will then try to bias the
        # the joint angles toward the goal joint configuration. The null space is 
        # the extra degrees of freedom the joints can move without affecting the
        # primary IK task.
        ikreq.use_nullspace_goal.append(True)
        # The nullspace goal can either be the full set or subset of joint angles
        goal = JointState()
        goal.name = ['right_j1', 'right_j2', 'right_j3']
        goal.position = [1, -1, 2]
        ikreq.nullspace_goal.append(goal)
        # The gain used to bias toward the nullspace goal. Must be [0.0, 1.0]
        # If empty, the default gain of 0.4 will be used
        ikreq.nullspace_gain.append(0.4)
    else:
        rospy.loginfo("Running Simple IK Service Client example.")

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return ret

    # Check if result valid, and type of seed ultimately used to get solution
    if (resp.result_type[0] > 0):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp.result_type[0], 'None')
        rospy.loginfo("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        rospy.loginfo("\nIK Joint Solution:\n%s", limb_joints)
        rospy.loginfo("------------------")
        rospy.loginfo("Response Message:\n%s", resp)
        ret.angle=list(resp.joints[0].position)
        ret.result=True
    else:
        rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
        rospy.logerr("Result Error %d", resp.result_type[0])
        return ret

    return ret

## go to joint angles calulated from IK
def go_to_angles(joint_angles_goal, speed_ratio_goal, accel_ratio_goal, timeout_goal):
    try:
        #rospy.init_node('go_to_joint_angles_py') # initialze once
        limb = Limb()
        traj = MotionTrajectory(limb = limb)

        wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=speed_ratio_goal,
                                         max_joint_accel=accel_ratio_goal)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        joint_angles = limb.joint_ordered_angles()

        waypoint.set_joint_angles(joint_angles = joint_angles)
        traj.append_waypoint(waypoint.to_msg())

        if len(joint_angles_goal) != len(joint_angles):
            rospy.logerr('The number of joint_angles must be %d', len(joint_angles))
            return None

        waypoint.set_joint_angles(joint_angles = joint_angles_goal)
        traj.append_waypoint(waypoint.to_msg())

        result = traj.send_trajectory(timeout=timeout_goal)
        if result is None:
            rospy.logerr('Trajectory FAILED to send')
            return

        if result.result:
            rospy.loginfo('Motion controller successfully finished the trajectory!')
        else:
            rospy.logerr('Motion controller failed to complete the trajectory with error %s',
                         result.errorId)
    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')




















