from funcs import *
## input parameter for solving IK
xyz=[0,0.3,0]
xyzw=[0.7,0.7,0.01,0.01]
## solve IK
rospy.init_node("rsdk_ik_service_client") # initialize the service (only once)
IK_result=ik_service_client(use_advanced_options=True, position_xyz=xyz, orientation_xyzw=xyzw)
if IK_result.result:
	rospy.loginfo("Advanced IK call passed!")
	print("joint angle:", IK_result.angle)
	go_to_angles(IK_result.angle, 0.1, 0.1, 1) # go to the soled joint angle
else:
	rospy.logerr("Advanced IK call FAILED")

