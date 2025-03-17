from urdfpy import URDF
from urdfpy import Link, Joint, Geometry, Box, Material, Visual, Inertial, Pose, Collision

robot = URDF(name="my_robot")

# Define links
base_link = Link(name="base_link", visual=Visual(geometry=Box(size=[1, 1, 1]), material=Material(name="blue", color=[0, 0, 1, 1])))
link1 = Link(name="link1", visual=Visual(geometry=Box(size=[1, 1, 1]), material=Material(name="red", color=[1, 0, 0, 1])),
             inertial=Inertial(mass=1, inertia=[1, 0, 0, 0, 1, 0, 0, 0, 1]))

# Define joint
joint1 = Joint(name="joint1", joint_type="revolute", parent=base_link, child=link1, origin=Pose(xyz=[0, 0, 1]), axis=[0, 1, 0])

# Add links and joints to robot model
robot.add_link(base_link)
robot.add_link(link1)
robot.add_joint(joint1)

# Save URDF to file
with open("my_robot.urdf", "w") as f:
    f.write(robot.to_xml_string())

print("URDF file created successfully: my_robot.urdf")