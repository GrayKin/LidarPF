##  This script is to help run your algorithm and visualize the result from it.

from scipy.io import loadmat
import numpy as np
import matplotlib.pyplot as plt
# from particleLocalization import particleLocalization


# This will load four variables: ranges, scanAngles, t, pose
# [1] t is K-by-1 array containing time in second. (K=3701)
#     You may not need to use time info for implementation.
# [2] ranges is 1081-by-K lidar sensor readings.
#     e.g. ranges(:,k) is the lidar measurement at time index k.
# [3] scanAngles is 1081-by-1 array containing at what angles the 1081-by-1 lidar
#     values ranges(:,k) were measured. This holds for any time index k. The
#     angles are with respect to the body coordinate frame.
# [4] M is a 2D array containing the occupancy grid map of the location
#     e.g. map(x,y) is a log odds ratio of occupancy probability at (x,y)


practice_mat = loadmat('./practice.mat')
M = practice_mat['M']
init_pose = practice_mat['init_pose']
pose = practice_mat['pose']
ranges = practice_mat['ranges']
scanAngles = practice_mat['scanAngles']
t = practice_mat['t']



# Set parameters
param = {}

# 1. Decide map resolution, i.e., the number of grids for 1 meter.
param['resol'] = 25
# 3. Indicate where you will put the origin in pixels
param['origin'] = [685,572]
param['init_pose'] = -init_pose

# # Plot LIDAR data
scanAngles = np.squeeze(scanAngles)
lidar_local = [ranges[:,0] * np.cos(scanAngles),-ranges[:,0] * np.sin(scanAngles)]

figure = plt.figure()
plt.plot(0,0)
plt.plot(lidar_local[0],lidar_local[1],'-')

# axis equal
plt.axis('equal')
plt.gca().invert_yaxis()
plt.xlabel('x')
plt.ylabel('y')
plt.grid()
plt.title('Lidar measurement in the body frame')
plt.show()
print('---')

# # %% Run algorithm
# # % Call your mapping function here.
# # % Running time could take long depending on the efficiency of your code.
# # % For a quicker test, you may take some hundreds frames as input arguments as
# # % shown.

num = 3700
# posePF = particleLocalization(ranges[:,0:num-1], scanAngles, M, param)

practice_answer = loadmat('./practice-answer.mat')


#  Plot final solution
#  The final grid map:
figure1 = plt.figure()


# Plot LIDAR data
lidar_global = np.zeros((ranges.shape[0], 2))
lidar_global[:,0] = ranges[:,0] * np.cos(scanAngles + pose[2][0]) + pose[0][0] * param['resol'] + param['origin'][0]
lidar_global[:,1] = -ranges[:,0] * np.sin(scanAngles + pose[2][0]) + pose[1][0] * param['resol'] + param['origin'][1]

plt.plot(lidar_global[:,0], lidar_global[:,1], 'g.')
plt.show()

# colormap('gray');
plt.axis('equal')
plt.imshow(M, cmap='gray')

plt.plot(pose[0,0:num-1]*param['resol']+param['origin'][0],pose[1,0:num-1]*param['resol']+param['origin'][1], 'r.-')
# plt.plot(posePF[0,0:num-1]*param['resol']+param['origin'][0], posePF[1,0:num-1]*param['resol']+param['origin'][1], 'y.-')
plt.show()
