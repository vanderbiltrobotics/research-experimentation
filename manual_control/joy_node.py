import rospy
from geometry msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data)
    twist = Twist()
    twist.linear.x = 4 * data.axes[1]
    twist.angular.z = 4*data.axes[0]
    pub.publish(twist)

# Intializes everything
def start():
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    pub = rospy.Publisher('turtle1/cmd_vel', Twist)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy2Turtle')
    rospy.spin()

if __name__ == '__main__':
    start()