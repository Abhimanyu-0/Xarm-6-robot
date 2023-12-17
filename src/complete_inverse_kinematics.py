x1 = 0
x2 = 0
x3 = 0
x4 = 25
x5 = 8
x6 = 0



T6 = test._endeff()
Desired = np.linalg.inv(T6)
norm_W = 500

while norm_W>0.2:

    for i in range(10):
        test99 = Robot(x1,x2,x3,x4,x5,x6)

        Current = test99._endeff()

        # I will get angular velocity first 
        Current_inv = np.linalg.inv(Current)
        MAT = scipy.linalg.logm(Current_inv@Desired)
        # For space jacobian 
        Vs = adjoint(Current)@MAT


        Rot = Vs[:3,0:3]

        omega = 0.5*(Rot - Rot.T)
        W = np.array([omega[2,1],omega[0, 2], omega[1, 0]])
        V = Vs[:3,3]
        vw  = np.append(V,W)
        VW = np.expand_dims(vw,axis=1)
        norm_W = np.linalg.norm(W)


        Jbian = test99._Jacobian()
        Jbian_psuedo = np.linalg.pinv(Jbian)
        Additive= Jbian_psuedo@VW
        alpha = 0.01

        x1 = x1 + alpha*Additive[0][0]
        x2 = x2 + alpha*Additive[1][0]
        x3 = x3 + alpha*Additive[2][0]
        x4 = x4 + alpha*Additive[3][0]
        x5 = x5 + alpha*Additive[4][0]
        x6 = x6 + alpha*Additive[5][0]

        print('x1: ',x1)
        print('x2: ',x2)
        print('x3: ',x3)
        print('x4: ',x4)
        print('x5: ',x5)
        print('x6: ',x6)
        print(norm_W)

        




