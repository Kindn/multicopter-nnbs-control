#!/usr/bin/python3

# auther: Peiyan Liu, NROS-Lab, HITSZ

import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import projections
import math

def calclulateMSE(v1, v2):
    n = v1.size
    return (np.sum((v1 - v2)**2) / n)

if __name__ == "__main__": 
    data_path = sys.argv[1]
    data = np.loadtxt(data_path)
    
    len = data.shape[0]
    
    t = data[:, 0]
    x_ref = data[:, 1]; y_ref = data[:, 2]; z_ref = data[:, 3]
    x_fdb = data[:, 4]; y_fdb = data[:, 5]; z_fdb = data[:, 6]
    vx_ref = data[:, 7]; vy_ref = data[:, 8]; vz_ref = data[:, 9]
    vx_fdb = data[:, 10]; vy_fdb = data[:, 11]; vz_fdb = data[:, 12]
    ax_ref = data[:, 13]; ay_ref = data[:, 14]; az_ref = data[:, 15]
    ax_fdb = data[:, 16]; ay_fdb = data[:, 17]; az_fdb = data[:, 18]
    
    # 3D
    fig_3d, axs_3d = plt.subplots(1, 2, subplot_kw={'projection': '3d'})
    axs_3d[0].plot(xs=x_ref, ys=y_ref, zs=z_ref, c='b')
    axs_3d[0].plot(xs=x_fdb, ys=y_fdb, zs=z_fdb, c='r')
    axs_3d[0].set_title(r"local position $(m)$")
    axs_3d[0].set_xlabel(r"$x$")
    axs_3d[0].set_ylabel(r"$y$")
    axs_3d[0].set_zlabel(r"$z$")
    axs_3d[0].legend(["reference", "feedback"])
    axs_3d[1].plot(xs=vx_ref, ys=vy_ref, zs=vz_ref, c='b')
    axs_3d[1].plot(xs=vx_fdb, ys=vy_fdb, zs=vz_fdb, c='r')
    axs_3d[1].set_title(r"local velocity $(m \cdot s^{-1})$")
    axs_3d[1].set_xlabel(r"$v_x$")
    axs_3d[1].set_ylabel(r"$v_y$")
    axs_3d[1].set_zlabel(r"$v_z$")
    axs_3d[1].legend(["reference", "feedback"])
    
    # 2D
    fig_2d, axs_2d = plt.subplots(2, 3, sharex=True)
    axs_2d[0, 0].plot(t, x_ref, c='b'); axs_2d[0, 0].plot(t, x_fdb, c='r')
    axs_2d[0, 0].set_title(r"local position x $(m)$")
    axs_2d[0, 0].set_xlabel(r"$t (s)$")
    axs_2d[0, 0].set_ylabel(r"$x$")
    axs_2d[0, 0].legend(["reference", "feedback"])
    axs_2d[0, 1].plot(t, y_ref, c='b'); axs_2d[0, 1].plot(t, y_fdb, c='r')
    axs_2d[0, 1].set_title(r"local position y $(m)$")
    axs_2d[0, 1].set_xlabel(r"$t (s)$")
    axs_2d[0, 1].set_ylabel(r"$y$")
    axs_2d[0, 1].legend(["reference", "feedback"])
    axs_2d[0, 2].plot(t, z_ref, c='b'); axs_2d[0, 2].plot(t, z_fdb, c='r')
    axs_2d[0, 2].set_title(r"local position z $(m)$")
    axs_2d[0, 2].set_xlabel(r"$t (s)$")
    axs_2d[0, 2].set_ylabel(r"$z$")
    axs_2d[0, 2].legend(["reference", "feedback"])
    
    axs_2d[1, 0].plot(t, vx_ref, c='b'); axs_2d[1, 0].plot(t, vx_fdb, c='r')
    axs_2d[1, 0].set_title(r"local velocity x $(m \cdot s^{-1})$")
    axs_2d[1, 0].set_xlabel(r"$t (s)$")
    axs_2d[1, 0].set_ylabel(r"$v_x$")
    axs_2d[1, 0].legend(["reference", "feedback"])
    axs_2d[1, 1].plot(t, vy_ref, c='b'); axs_2d[1, 1].plot(t, vy_fdb, c='r')
    axs_2d[1, 1].set_title(r"local velocity y $(m \cdot s^{-1})$")
    axs_2d[1, 1].set_xlabel(r"$t (s)$")
    axs_2d[1, 1].set_ylabel(r"$v_y$")
    axs_2d[1, 1].legend(["reference", "feedback"])
    axs_2d[1, 2].plot(t, vz_ref, c='b'); axs_2d[1, 2].plot(t, vz_fdb, c='r')
    axs_2d[1, 2].set_title(r"local velocity z $(m \cdot s^{-1})$")
    axs_2d[1, 2].set_xlabel(r"$t (s)$")
    axs_2d[1, 2].set_ylabel(r"$v_z$")
    axs_2d[1, 2].legend(["reference", "feedback"])
    
    rmse_pos = (calclulateMSE(x_ref, x_fdb) + calclulateMSE(y_ref, y_fdb) + calclulateMSE(z_ref, z_fdb))**0.5
    rmse_vel = (calclulateMSE(vx_ref, vx_fdb) + calclulateMSE(vy_ref, vy_fdb) + calclulateMSE(vz_ref, vz_fdb))**0.5
    
    print("position RMSE: " + str(rmse_pos))
    print("velocity RMSE: " + str(rmse_vel))
    
    plt.show()
