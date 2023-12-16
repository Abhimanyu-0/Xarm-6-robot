#Takes in position arrays which specify how far from the tool frame is the force on wrench applied and F denotes the force
def inverse_dynamics(p,F):
    # The above are the distances measured in the xyz frame 
    tau_x = p[1]*F[2] - (p[2]*F[1])   #taux = ry*Fz - rz*Fy
    tau_y = (p[2]*F[0]) - (p[0]*F[2]) #tauy = rz*Fx - rx*Fz 
    tau_z = (p[0]*F[1]) - (p[1]*F[0]) #tauz = rx*Fy - ry*Fx   

#Testing function 
p = np.array([1,1,1])
F = np.array([5,10,0])
Force = inverse_dynamics(p,F)


Jtoolw = test._Jacobian()[3:6]
Force_cartes = np.expand_dims(Force,axis=1)
tau_joints = -(Jtoolw.T@Force_cartes) #Gets torque that should be applied at joints to oppose force 

