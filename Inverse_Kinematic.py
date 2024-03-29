import numpy as np

# Target end effector point
x = 20
y = 20
z = 30

# Length of the links
a_1 = 20
a_2 = 20
a_3 = 10

# Calculate theta_1
theta_1 = np.degrees(np.arctan2(y, x))

# Calculate phi_2
r_1 = np.sqrt(x**2 + y**2)
r_2 = z - a_1
phi_2 = np.arctan2(r_2, r_1)

# Calculate theta_2
numerator = a_3**2 - a_2**2 - r_1**2 - r_2**2
denominator = -2 * a_2 * np.sqrt(r_1**2 + r_2**2)
phi_1 = np.arccos(np.clip(numerator / denominator, -1, 1))
theta_2 = np.degrees(phi_1 - phi_2)

# Calculate theta_3
numerator = r_1**2 + r_2**2 - a_2**2 - a_3**2
denominator = -2 * a_2 * a_3
phi_3 = np.arccos(np.clip(numerator / denominator, -1, 1))
theta_3 = np.degrees(np.pi - phi_3)

print("Theta_1:", theta_1)
print("Theta_2:", theta_2)
print("Theta_3:", theta_3)
