import csv
import numpy as np
import matplotlib.pyplot as plt

with open("arm_angles.csv", "r") as f:
	reader = csv.reader(f)
	data = list(reader)

data = np.array(data)

#print data[:,1]
print data.shape[0]

plt.plot(data[:,0], label="1st arm")
plt.plot(data[:,1], label="2nd arm")
plt.plot(data[:,2], label="3rd arm")
plt.xlabel("Time step")
plt.ylabel("Rotation angle [rad]")
plt.legend(loc=2)
plt.show()