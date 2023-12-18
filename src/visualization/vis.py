import pybullet as p

#init
p.connect(p.GUI)

#Add your own path
robot_urdf_path = "/Users/abhimanyusuthar/anaconda3/Robotics/gitrepo/bullet3/examples/pybullet/gym/pybullet_data/xarm/xarm6_robot.urdf"
robot_id = p.loadURDF(robot_urdf_path, useFixedBase=Trueimport time
initial_joint_angles = [m.radians(0), m.radians(30), m.radians(0), m.radians(-180), m.radians(0), m.radians(90),m.radians(0)] # This is the home configuration(according to my DH table)
for i in range(0,p.getNumJoints(robot_id)-1):
    p.resetJointState(robot_id, i, initial_joint_angles[i])

for i in range(500):
    p.stepSimulation()
    time.sleep(1/100) )
