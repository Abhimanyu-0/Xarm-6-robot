# Need to finetune the loop parameters to make sure algorithm converges

test = Robot(0,0,0,0,10,0) #instantiating robot class 

end = test._endeff()[:,3] #Getting the last column of the transformation matrix for end-effector

#Getting the position of the end-effector in a 3x1 vector
posori = np.array([end[0],end[1],end[2]])
POSORI = np.expand_dims(posori, axis =0).transpose()

#The intialization of angles for peforming numerical IK via Newton-raphson method
st1 = 9
st2 = 9
st3 = 9
st4 = 9
st5 = 9
st6 = 9

Error_P = [500] # Initializing random value of Error_P
for i in range(0,100):

    test2 = Robot(st1,st2,st3,st4,st5,st6)
    end2 = test2._endeff()[:,3]
    End2 = np.expand_dims(end2[:3],axis=0).transpose()
    err_P = np.subtract(POSORI[0:3],End2)
    Err_P = np.linalg.norm(err_P)
    Error_P.insert(i,Err_P) 

    alpha = 0.1
    if (Error_P[i]<Error_P[i-1]): 
        err_P = np.subtract(POSORI[0:3],End2)
        Err_P = np.linalg.norm(err_P)

	#Getting the first three rows of the jacobian matrix
        J1 = test2._Jacobian()
        Jv = J1[0:3]
        Jv_pseudo = np.linalg.pinv(Jv)

	# Calculating norm error to cross verify results
	ERR = np.expand_dims(Err_P, axis =0).transpose()
        norm_error = np.linalg.norm(ERR)

	D_theta = Jv_pseudo@err_P #Change in theta 


	# Update of initial and then subsequent guesses
        st1 = st1 + alpha*(D_theta[0][0])
        st2 = st2 + alpha*(D_theta[1][0])
        st3 = st3 + alpha*(D_theta[2][0])
        st4 = st4 + alpha*(D_theta[3][0])
        st5 = st5 + alpha*(D_theta[4][0])
        st6 = st6 + alpha*(D_theta[5][0])

    else:
        st1 = st1 + alpha*(D_theta[0][0])/2
        st2 = st2 + alpha*(D_theta[1][0])/2
        st3 = st3 + alpha*(D_theta[2][0])/2
        st4 = st4 + alpha*(D_theta[3][0])/2
        st5 = st5 + alpha*(D_theta[4][0])/2
        st6 = st6 + alpha*(D_theta[5][0])/2
        print("Changes made and result is: ", norm_error) # This is done to ensure the algorithm converges
