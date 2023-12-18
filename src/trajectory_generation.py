# Define initial and final joint angles (test case)
initial = np.array([5,6,7,18,9,16])
final = np.array([15,14,13,12,11,10])

# Display initial and final joint angles
print("Initial Joint Angles:", initial)
print("Final Joint Angles:", final)

# Define trajectory duration and time parameters
tf = 5
t0 = 0
num_points = 100

# Generate time vector
T = 5
t = np.linspace(0,T,100)
a = initial 
a1 =0
a2 = (3/tf**2)*(final -initial)
a3 = (-2/tf**3)*(final -initial)
#trajectory = a + a1*t + a2*t**2 + a3*t**3

# Initialize joint trajectory array
joint_trajectory = np.zeros((num_points, len(initial))) 

# Initialize list to store maximum accelerations for each joint
max_acc = []

# Generate joint trajectory using cubic polynomial for each joint angle
for i in range(len(initial)):
        # Calculate coefficients for cubic polynomial
        a = initial[i]
        b = 0
        c = 3 * (final[i] - initial[i]) / (T**2)
        d = 2 * (initial[i] - final[i]) / (T**3)
        
        # Calculate and store maximum acceleration for each joint
        max_acc.insert(i,(6/(T**2))*(final[i]-initial[i])) # maximum accleration

        # Evaluate cubic polynomial for each time point
        joint_trajectory[:, i] = a + b * t+ c * t**2 + d * t**3

# Plot the joint trajectory
plt.figure(figsize=(10, 6))
for i in range(joint_trajectory.shape[1]):
    plt.plot(t, joint_trajectory[:, i], label=f'Joint {i+1}')

plt.xlabel('time')
plt.ylabel('thetas')
plt.title('Joint Trajectory Generation')
plt.legend()


for i, angle in enumerate(initial):
    plt.text(0.5, initial[i], f'$\\theta_{i+1}$', fontsize=12, color='black', ha='right')
