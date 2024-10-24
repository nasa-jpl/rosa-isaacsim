#  Copyright (c) 2024. Jet Propulsion Laboratory. All rights reserved.
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#  https://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

import rospy
import math
from typing import Literal
from langchain.agents import tool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

cmd_vel_pub = None
update_output_box = None
is_running = False

def set_cmd_vel_pub(pub):
    global cmd_vel_pub
    cmd_vel_pub = pub

def set_update_output_box(update_box_ref):
    global update_output_box
    update_output_box = update_box_ref

def set_is_running(value: bool):
    global is_running
    is_running = value

def get_is_running():
    global is_running
    return is_running


@tool
async def move(direction: Literal["forward", "backward"], distance: float):
    """
    Move the robot in the specified direction by the specified distance.

    :param direction: The direction to move the robot in. Can be one of "forward", "backward".
    :param distance: The distance to move the robot by.
    """
    global cmd_vel_pub, is_running, update_output_box
    print(f"Moving {direction} by {distance} meters...")
    update_output_box.text = "Moving the robot...\n"

    twist = Twist(
        linear=Vector3(x=0, y=0, z=0),
        angular=Vector3(x=0, y=0, z=0)
    )

    if direction == "forward":
        twist.linear.x = 0.5
    elif direction == "backward":
        twist.linear.x = -0.5

    print(f"Subscribing to /odom topic...")
    update_output_box.text = "Getting initial position..."
    odom = rospy.wait_for_message("/odom", Odometry, timeout=3)
    print(f"Initial position: {odom.pose.pose.position.x}, {odom.pose.pose.position.y}")
    start_x = odom.pose.pose.position.x
    start_y = odom.pose.pose.position.y

    rate = rospy.Rate(30)

    update_output_box.text = "Moving the robot...\n"
    while is_running:
        cmd_vel_pub.publish(twist)
        odom = rospy.wait_for_message("/odom", Odometry)
        current_x = odom.pose.pose.position.x
        current_y = odom.pose.pose.position.y
        current_distance = ((current_x - start_x) ** 2 + (current_y - start_y) ** 2) ** 0.5

        updated_distance = current_distance if direction == "forward" else -current_distance
        update_output_box.text = f"Distance traversed: {updated_distance}\n"

        if current_distance >= distance:
            print(f"Final position: {current_x}, {current_y}")
            break

        rate.sleep()

    twist.linear.x = 0
    cmd_vel_pub.publish(twist)

    if not is_running:
        return "Task was deliberately stopped by the user. You should ask follow-up questions if necessary."
    return f"Moved {direction} by {distance} meters."

@tool
def turn(direction, angle, unit="radians"):
    """
    Turn the robot in the specified direction by the specified angle.

    :param direction: The direction to turn the robot in. Can be one of "left", "right".
    :param angle: The angle to turn the robot.
    :param unit: The unit of the angle given. Can be one of "radians", "degrees".
    """
    global cmd_vel_pub, is_running, update_output_box
    rate = rospy.Rate(100)

    twist = Twist()
    twist.angular.z = 0.5 if direction == "left" else -0.5

    # Initialize start_yaw
    update_output_box.text = "Getting initial orientation..."
    odom = rospy.wait_for_message("/odom", Odometry)
    start_yaw = math.atan2(2.0 * (odom.pose.pose.orientation.w * odom.pose.pose.orientation.z + odom.pose.pose.orientation.x * odom.pose.pose.orientation.y),
                           1.0 - 2.0 * (odom.pose.pose.orientation.y * odom.pose.pose.orientation.y + odom.pose.pose.orientation.z * odom.pose.pose.orientation.z))

    # Convert target angle to radians if necessary
    if unit == "degrees":
        target_angle = math.radians(angle)
    else:
        target_angle = angle

    current_angle = 0.0

    while is_running:
        cmd_vel_pub.publish(twist)

        odom = rospy.wait_for_message("/odom", Odometry)
        current_yaw = math.atan2(2.0 * (odom.pose.pose.orientation.w * odom.pose.pose.orientation.z + odom.pose.pose.orientation.x * odom.pose.pose.orientation.y),
                                 1.0 - 2.0 * (odom.pose.pose.orientation.y * odom.pose.pose.orientation.y + odom.pose.pose.orientation.z * odom.pose.pose.orientation.z))
        
        # Calculate the shortest angle difference
        delta_yaw = current_yaw - start_yaw
        delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))

        current_angle += delta_yaw
        start_yaw = current_yaw  # Update start_yaw for the next iteration

        update_output_box.text = f"Angle turned: {math.degrees(current_angle)} degrees\n"

        # Compare the current angle turned with the desired angle in radians
        if abs(current_angle) >= target_angle:
            print(f"Current angle: {math.degrees(current_angle)} degrees, Target angle: {math.degrees(target_angle)} degrees")
            break

        rate.sleep()

    twist.angular.z = 0
    cmd_vel_pub.publish(twist)

    if not is_running:
        return "Task was deliberately stopped by the user. You should ask follow-up questions if necessary."

    return f"Turned {direction} by {angle} {unit}."
