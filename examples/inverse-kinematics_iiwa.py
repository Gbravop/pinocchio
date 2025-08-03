import numpy as np
import pinocchio
from numpy.linalg import norm, solve

urdf_model_path = "urdfs/iiwa.urdf"
model, _, _ = pinocchio.buildModelsFromUrdf(urdf_model_path)
data = model.createData()

JOINT_ID = 7
Frame_ID = model.getFrameId("iiwa_link_7")
print(f"Joint ID: {JOINT_ID}")
oMdes = pinocchio.SE3(np.eye(3), np.array([0.5, 0.0, 0.5]))

q = pinocchio.neutral(model)
print("Initial joint configuration:", q.flatten().tolist())
eps = 1e-4
IT_MAX = 1000
DT = 1e-1
damp = 1e-12

q_max = np.array([170, 120, 120, 120, 120, 120, 175]) * np.pi / 180.0
q_min = -q_max

i = 0
while True:
    pinocchio.forwardKinematics(model, data, q)
    iMd = data.oMi[JOINT_ID].actInv(oMdes)
    # print("End effector pose:", np.array(data.oMi[7].translation))
    err = pinocchio.log(iMd).vector  # in joint frame
    if norm(err) < eps and np.all(q <= q_max) and np.all(q >= q_min):
        success = True
        break
    if i >= IT_MAX:
        success = False
        break
    # J = pinocchio.computeFrameJacobian(model, data, q, Frame_ID, pinocchio.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    J = pinocchio.computeJointJacobian(model, data, q, JOINT_ID)  # in joint frame
    J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
    v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
    q = pinocchio.integrate(model, q, v * DT)
    if not i % 10:
        print(f"{i}: error = {err.T}")
    i += 1

if success:
    print("Convergence achieved!")
else:
    print(
        "\n"
        "Warning: the iterative algorithm has not reached convergence "
        "to the desired precision"
    )

PI = np.pi
print(f"\nresult in radians: {q.flatten().tolist()}")
print(f"\nresult in degrees: {(np.array(q.flatten().tolist())*180/PI).tolist()}")
print(f"\nfinal error: {err.T}")
