from urdfpy import URDF
from urdfpy import Link, Joint
import numpy as np

# Create links
base_link = Link(name="base_link", visuals=None, inertial=None, collisions=None)
link1 = Link(name="link1", visuals=None, inertial=None, collisions=None)
link2 = Link(name="link2", visuals=None, inertial=None, collisions=None)

# Create joints
joint1 = Joint(
    name="joint1",
    joint_type="revolute",
    parent=base_link,
    child=link1,
    origin=np.array([0, 0, 0]),
    axis=np.array([0, 0, 1]),
)

joint2 = Joint(
    name="joint2",
    joint_type="revolute",
    parent=link1,
    child=link2,
    origin=np.array([1, 0, 0]),
    axis=np.array([0, 0, 1]),
)

# Create the robot
robot = URDF(name="my_robot", links=[base_link, link1, link2], joints=[joint1, joint2])

# Save to a URDF file
robot.save("my_robot.urdf")
