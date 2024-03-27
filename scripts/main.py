#!/usr/bin/python3
import socket
import rospy
from geometry_msgs.msg import Twist

# Create TCP/IP server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('192.168.149.1', 2030)) # IP and Port (192.168.149.54)
s.listen(1) # Only 1 client at the time

def receive_from_wifi_esp32():
    try:
        client = s.accept()[0] # Accept client connection
    except s.timeout:
        pass
    while True:
        data = client.recv(32).decode('utf-8') # Decode received data
        data = data.replace('\r', '').replace('\n', '') # Clean data
        if data: 
            velocity = [float(val) for val in data.split(',')] # Get velocities list as floats
        else: 
            break # No more data

    client.close() # End current client connection
    return velocity

# Stop Condition
def stop():
    # Stop message
    print("Stopping")

if __name__ == '__main__':
    # Initialise and Setup node
    rospy.init_node("Controller")
    rospy.on_shutdown(stop)
    
    hz = 10 # Frequency (Hz)
    rate = rospy.Rate(hz)

    # Publish the velocities for the ominidirectional vehicle
    omniVel = rospy.Publisher("/jetauto_controller/cmd_vel", Twist, queue_size = 1)

    omniVelocities = Twist() # Linear and angular velocities (x, y, z)
    print("The Velocity Controller is Running")
    while not rospy.is_shutdown():
        try:
            # Convert velocity for the 4 wheels or publish to high level topic
            [omniVelocities.linear.x, omniVelocities.linear.y, omniVelocities.angular.z] = receive_from_wifi_esp32()
            omniVel.publish(omniVelocities) # Publish the velocities
            rate.sleep()
        # Manage exceptions
        except ValueError:
            omniVel.publish(Twist()) # Send velocities on 0.0
            print(ValueError)
            exit()