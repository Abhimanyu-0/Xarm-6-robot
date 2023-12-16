#initialised random angles to test program
f = Robot(15,15,15,15,15,15)
i = Robot(0,0,0,0,0,0)

initial = np.array([5,6,7,8,9,3])
final = np.array([15,14,13,12,11,10])
print(initial)
print(final)
tf = 5
t0 = 0
num_points = 100

T = 5
t = np.linspace(0,T,100)
a = initial 
a1 =0
a2 = (3/tf**2)*(final -initial)
a3 = (-2/tf**3)*(final -initial)
#trajectory = a + a1*t + a2*t**2 + a3*t**3

joint_trajectory = np.zeros((num_points, len(initial))) 
max_acc = []
for i in range(len(initial)):
        # Calculate coefficients for cubic polynomial
        a = initial[i]
        b = 0
        c = 3 * (final[i] - initial[i]) / (T**2)
        d = 2 * (initial[i] - final[i]) / (T**3)
        max_acc.insert(i,(6/(T**2))*(final[i]-initial[i])) # maximum accleration

        # Evaluate cubic polynomial for each time point
        joint_trajectory[:, i] = a + b * t+ c * t**2 + d * t**3

# Visualizes trajectory vs time (verifies its smooth)
plt.figure(figsize=(10, 6))
for i in range(joint_trajectory.shape[1]):
    plt.plot(t, joint_trajectory[:, i], label=f'Joint {i+1}')

plt.xlabel('time')
plt.ylabel('thetas')

