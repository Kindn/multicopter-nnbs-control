import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import math

def eulerAngles2rotationMat(theta, format='degree'):
    if format == 'degree':
        theta = [i * math.pi / 180.0 for i in theta]
 
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])
 
    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])
 
    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])
    R = np.dot(R_x, np.dot(R_y, R_z))
    
    return R

if __name__ == "__main__":
    data_path = sys.argv[1]
    data = np.loadtxt(data_path)
    
    len = data.shape[0]
    
    for i in range(len):
        data[i, 1:4] = R.from_matrix(eulerAngles2rotationMat(data[i, 1:4], format="rad")).as_euler("XYZ")
        data[i, 5:8] = R.from_matrix(eulerAngles2rotationMat(data[i, 5:8], format="rad")).as_euler("XYZ")
    
    t_est = data[:, 0]
    r_est = data[:, 1]
    p_est = data[:, 2]
    y_est = data[:, 3]
    
    t_mav = data[:, 4]
    r_mav = data[:, 5]
    p_mav = data[:, 6]
    y_mav = data[:, 7]
    
    fig, axs = plt.subplots(3, 1, sharex=True)
    ax0 = axs[0]
    ax1 = axs[1]
    ax2 = axs[2]
    
    ax0.plot(t_est, r_est)
    ax0.plot(t_mav[:int(2 * len / 3)], r_mav[:int(2 * len / 3)])
    ax0.set_ylabel("roll/rad", fontsize=20)
    ax0.legend(["estimated result", "px4 feedback"])
    
    ax1.plot(t_est, p_est)
    ax1.plot(t_mav[:int(2 * len / 3)], p_mav[:int(2 * len / 3)])
    ax1.set_ylabel("pitch/rad", fontsize=20)
    ax1.legend(["estimated result", "px4 feedback"])
    
    ax2.plot(t_est, y_est)
    ax2.plot(t_mav[:int(2 * len / 3)], y_mav[:int(2 * len / 3)])
    ax2.set_ylabel("yaw/rad", fontsize=20)
    ax2.legend(["estimated result", "px4 feedback"])
    ax2.set_xlabel("t/sec")
    
    plt.show()
    