import numpy as np
import pinocchio
from numpy.linalg import norm, solve

urdf_model_path = "urdfs/iiwa.urdf"
model = pinocchio.buildModelFromUrdf(urdf_model_path)
data = model.createData()

JOINT_ID = 7
Frame_ID = model.getFrameId("iiwa_link_7")

# q = pinocchio.neutral(model)
eps = 1e-4
IT_MAX = 10000
DT = 1e-1
damp = 1e-12

q_max = np.array([170, 120, 120, 120, 120, 120, 175]) * np.pi / 180.0
print("Max joint angles (radians):", q_max.flatten().tolist())
q_min = -q_max
print("Min joint angles (radians):", q_min.flatten().tolist())

goals = [np.array([0.5, -.1865, 0.5]),
         np.array([0.5, 0.3, 0.2]),
         np.array([0.3, 0.3, 0.8]),
         np.array([0.6, -0.4, 0.2]),
         np.array([0., -0.45, 0.8])]

for goal in goals:
    oMdes = pinocchio.SE3(np.eye(3), goal)
    print("=====================================================================================")
    print(f"Desired end-effector position: {goal}")

    q = pinocchio.neutral(model)  # reset to neutral configuration
    print("Initial joint configuration:", q.flatten().tolist())
    
    it = 0
    while True:
        pinocchio.forwardKinematics(model, data, q)
        iMd = data.oMi[JOINT_ID].actInv(oMdes)
        err = iMd.translation
        if norm(err) < eps:
            success = True
            break
        if it >= IT_MAX:
            success = False
            break
        J = pinocchio.computeFrameJacobian(model, data, q, Frame_ID, pinocchio.ReferenceFrame.WORLD)
        # J = pinocchio.computeJointJacobian(model, data, q, JOINT_ID)  # in joint frame
        J = -J[:3, :]  # linear part of the Jacobian
        v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(3), err))
        q = pinocchio.integrate(model, q, v * DT)
        if not it % 1000:
            print(f"{it}: error = {err.T}")
        it += 1

    PI = np.pi
    revs = (np.array(q.flatten().tolist())/(2*PI)).tolist()
    revs_fraction = np.array(revs) - np.round(np.array(revs))
    revsf_in_rads = (revs_fraction*2*PI).tolist()
    revsf_in_degs = (revs_fraction*360).tolist()
    print(f"\n 1 revolute reference: {2*PI}")
    print(f"\n max q reference: {170*PI/180}")
    print(f"\nresult: {q.flatten().tolist()}")
    print(f"\nresult revs.: {revs}")
    print(f"\nresult revs. integer part: {(np.round(np.array(revs))).tolist()}")
    print(f"\nresult revs. frac: {revs_fraction.tolist()}")
    print(f"\nresult revsf_in_rads: {revsf_in_rads}")
    print(f"\nresult revsf_in_degs: {revsf_in_degs}")
    print(f"\nfinal error: {err.T}")

    # Save q from each iteration to the same CSV file:
    with open("iiwa_ik_desired_q.csv", "a") as f:
        f.write(",".join(str(x) for x in revsf_in_rads) + "\n")

    if success:
        print("\nConvergence achieved!")
    else:
        print(
            "\nWarning: the iterative algorithm has not reached convergence to "
            "the desired precision"
        )


