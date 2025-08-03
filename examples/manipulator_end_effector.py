import numpy as np
import pinocchio as pin

# Load the model from a URDF file
# Change to your own URDF file here, or give a path as command-line argument
urdf_model_path = "urdfs/iiwa.urdf"
model, _, _ = pin.buildModelsFromUrdf(urdf_model_path)

# Build a data frame associated with the model
data = model.createData()

# Define the joint configuration (replace with your actual joint values)
q = pin.neutral(model) # Neutral configuration as example
# q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6]) # Example with specific joint values

# Perform forward kinematics to update the model data
pin.forwardKinematics(model, data, q)

# Specify the end-effector frame ID (replace with your actual frame ID)
end_effector_frame_id = model.getFrameId("iiwa_link_7") # Gets the first end-effector ID

#
pin.updateFramePlacement(model,data,end_effector_frame_id)

# Get the end-effector position
end_effector_position = data.oMf[end_effector_frame_id].translation

# Print the end-effector position
print("Neutral position:", q)
print("End-effector position:", end_effector_position)