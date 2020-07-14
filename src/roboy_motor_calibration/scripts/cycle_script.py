import rospy
import roboy_middleware_msgs.msg

rospy.init_node("cycle_script")

pos = 0
initial_pos = 0
inital_pose_received = False

def MotorStatus(msg):
    global pos
    global initial_pos
    global inital_pose_received
    pos = msg.position[11]
    if not inital_pose_received:
        initial_pos = pos
        inital_pose_received = True

    # rospy.loginfo_throttle(1,pos)

pub = rospy.Publisher('roboy/middleware/MotorCommand', roboy_middleware_msgs.msg.MotorCommand, queue_size=1)
rospy.Subscriber("roboy/middleware/MotorStatus", roboy_middleware_msgs.msg.MotorStatus, MotorStatus)
up = True

cycle = 0
max_cycles = 1000

while not rospy.is_shutdown():
    if cycle>max_cycles:
        break
    if inital_pose_received:
        msg = roboy_middleware_msgs.msg.MotorCommand()
        msg.id = 3
        msg.motors = [11]
        if pos>initial_pos+450000 and up:
            up = False
        if pos<initial_pos+50000 and not up:
            rospy.loginfo_throttle(1,"cycle %d/%d"%(cycle,max_cycles))
            cycle = cycle + 1
            up = True
        if up:
            msg.set_points = [3000]
        else:
            msg.set_points = [600]
        pub.publish(msg)
    else:
        rospy.loginfo_throttle(1,"waiting for initial position message")

rospy.loginfo("done")