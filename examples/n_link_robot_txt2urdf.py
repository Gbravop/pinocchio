from urdfpy import URDF
from urdfpy import Link, Joint
import numpy as np

# Number of links:
N = 3

# Create file
sn1 = "urdfs/NL_link_pendulum.urdf"
sn2 = sn1.replace("L",str(N)) 
file = open(sn2, "w")

# File name
s1 = "<robot name=\"N_link_manipulator\">\n"
s2 = s1.replace("N", str(N))
file.write(s2)
file.write("\n")

# Create links
file.write("  <link name=\"base_link\">\n")
file.write("        <inertial>\n")
file.write("		    <origin xyz = \"0 0 0\" />\n")
file.write("		    <mass value = \"1.0\" />\n")
file.write("		    <inertia ixx = \"0.5\" iyy = \"0.5\" izz = \"0.5\" ixy = \"0\" ixz = \"0\" iyz = \"0\" />\n")
file.write("	    </inertial>\n")
file.write("        <visual>\n")
file.write("		    <origin rpy=\"0 0 0\" xyz = \"0 0 0\" />\n")
file.write("		    <geometry>\n")
file.write("			    <box size = \"0.1 0.1 0.1\" />\n")
file.write("		    </geometry>\n")
file.write("		    <material name = \"gray A\">\n")
file.write("			    <color rgba = \"0.1 0.1 0.1 1\" />\n")
file.write("		    </material>\n")
file.write("		</visual>\n")
file.write("  </link>\n")
file.write("\n")

for j in range(1,N+1):
    sl1 = "  <link name=\"link_N\">\n"
    sl2 = sl1.replace("_N",str(j))
    file.write(sl2)
    file.write("    <visual>\n")
    file.write("        <geometry>\n")
    file.write("            <cylinder length=\"0.5\" radius=\"0.05\"/>\n")
    file.write("        </geometry>\n")
    file.write("        <origin rpy=\"1.57075 0 0\" xyz=\"0 -0.25 0\"/>\n")
    file.write("        <material name = \"gray B\">\n")
    file.write("		    <color rgba = \"0.3 0.3 0.3 1\" />\n")
    file.write("        </material>\n")
    file.write("    </visual>\n")
    file.write("    <inertial>\n")
    file.write("        <mass value=\"1\"/>\n")
    file.write("        <origin xyz=\"0 -0.25 0\"/>\n")
    file.write("        <inertia ixx = \"0.0208\" iyy = \"0.0208\" izz = \"0.0208\" ixy = \"0\" ixz = \"0\" iyz = \"0\" />\n")
    file.write("    </inertial>\n")
    file.write("  </link>\n")
    file.write("\n")

for j in range(1,N+1):
    # Create joints
    if (j==1):
        file.write("  <joint name=\"base_to_link1\" type=\"revolute\">\n")
        file.write("    <parent link=\"base_link\"/>\n")
        file.write("    <child link=\"link1\"/>\n")
    else:
        sj1 = "  <joint name=\"link_p_to_link_c\" type=\"revolute\">\n"
        sj2 = sj1.replace("_p",str(j-1))
        sj3 = sj2.replace("_c",str(j))
        file.write(sj3)
        sj4 = "    <parent link=\"link_p\"/>\n"
        sj5 = sj4.replace("_p",str(j-1))
        file.write(sj5)
        sj6 = "    <child link=\"link_c\"/>\n"
        sj7 = sj6.replace("_c",str(j))
        file.write(sj7)

    file.write("    <axis xyz=\"0 0 1\"/>\n")
    file.write("    <origin xyz=\"0 -0.05 0\"/>\n")
    file.write("    <limit effort = \"1000.0\" lower=\"-3.14\" upper = \"3.14\" velocity=\"0.5\"/>\n")
    file.write("  </joint>\n")
    file.write("\n")

file.write("</robot>\n")

# Create txt file
file.close()

# Save to a URDF file
