from pathlib import Path
from sys import argv
 
import pinocchio as pin
 
# Load the urdf model
model = pin.RobotWrapper.BuildFromMJCF("model/trs_so_arm100/so_arm100.xml").model
# model = pin.buildModelFromUrdf("model/so_arm100_description/so100.urdf")
print("model name: " + model.name)
print("lowerLimits: " + str(model.lowerPositionLimit))
print("upperLimits: " + str(model.upperPositionLimit))
 
# Create data required by the algorithms
data = model.createData()
 
# Sample a random configuration
q = pin.randomConfiguration(model)
print(f"q: {q.T}")
 
# Perform the forward kinematics over the kinematic tree
pin.forwardKinematics(model, data, q)
 
# Print out the placement of each joint of the kinematic tree
for name, oMi in zip(model.names, data.oMi):
    print("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat))