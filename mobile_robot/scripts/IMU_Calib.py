from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250
import numpy as np
import csv,datetime
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import time
def temp():
    sample_counts = 0
    data_non_calib = {'ax':[], 'ay':[], 'az':[], 'wx':[],'wy':[],'wz':[]}
    data_calib = {'ax':[], 'ay':[], 'az':[], 'wx':[],'wy':[],'wz':[]}
    while sample_counts <= 100:
        ax,ay,az = mpu.readAccelerometerMaster()
        data_non_calib['ax'].append(ax)
        data_non_calib['ay'].append(ay)
        data_non_calib['az'].append(az)
        wx,wy,wz = mpu.readGyroscopeMaster()
        data_non_calib['wx'].append(wx)
        data_non_calib['wy'].append(wy)
        data_non_calib['wz'].append(wz)
        time.sleep(0.1)
    sample_counts = 0
    while sample_counts <= 100:
        mpu.calibrate()
        mpu.configure()
        ax,ay,az = mpu.readAccelerometerMaster()
        data_calib['ax'].append(ax)
        data_calib['ay'].append(ay)
        data_calib['az'].append(az)
        wx,wy,wz = mpu.readGyroscopeMaster()
        data_calib['wx'].append(wx)
        data_calib['wy'].append(wy)
        data_calib['wz'].append(wz)
        time.sleep(0.1)



def accel_cal():
    print("-"*50)
    print("Accelerometer Calibration")
    mpu_offsets = [[],[],[]] # offset array to be printed
    axis_vec = ['z','y','x'] # axis labels
    cal_directions = ["upward","downward","perpendicular to gravity"] # direction for IMU cal
    cal_indices = [2,1,0] # axis indices
    for qq,ax_qq in enumerate(axis_vec):
        ax_offsets = [[],[],[]]
        print("-"*50)
        for direc_ii,direc in enumerate(cal_directions):
            input("-"*8+" Press Enter and Keep IMU Steady to Calibrate the Accelerometer with the -"+\
              ax_qq+"-axis pointed "+direc)
            [mpu.getAllData() for ii in range(0,cal_size)] # clear buffer between readings
            mpu_array = []
            while len(mpu_array)<cal_size:
                try:
                    ax,ay,az = mpu.readAccelerometerMaster()
                    mpu_array.append([ax,ay,az]) # append to array
                except:
                    continue
            ax_offsets[direc_ii] = np.array(mpu_array)[:,cal_indices[qq]] # offsets for direction

        # Use three calibrations (+1g, -1g, 0g) for linear fit
        #popts,_ = curve_fit(accel_fit,np.append(np.append(ax_offsets[0],
         #                        ax_offsets[1]),ax_offsets[2]),
        #         np.append(np.append(1.0*np.ones(np.shape(ax_offsets[0])),
        #            -1.0*np.ones(np.shape(ax_offsets[1]))),
        #                0.0*np.ones(np.shape(ax_offsets[2]))),
        #                    maxfev=10000)
        mpu.calibrate()
        mpu.configure()
        mpu_offsets[cal_indices[qq]] = mpu.abias# place slope and intercept in offset array
    print('Accelerometer Calibrations Complete')
    return mpu_offsets

    
    
    
    
if __name__ == "__main__":
    mpu = MPU9250(
        address_ak=  AK8963_ADDRESS,
        address_mpu_master = MPU9050_ADDRESS_68,
        address_mpu_slave= None,
        bus=1,
        gfs=GFS_1000,
        afs=AFS_8G,
        mfs=AK8963_BIT_16,
        mode=AK8963_MODE_C100HZ
    )

    mpu.configure()
    start_bool = True
    if not start_bool:
        print("IMU not Started - Check Wiring and I2C")
    else:
        #
        ###################################
        # Accelerometer Gravity Calibration
        ###################################
        #
        accel_labels = ['a_x','a_y','a_z'] # gyro labels for plots
        cal_size = 100 # number of points to use for calibration 
        #accel_coeffs = accel_cal() # grab accel coefficients
#
        ###################################
        # Record new data 
        ###################################
        #
        data = np.array([mpu.readAccelerometerMaster() for ii in range(0,cal_size)]) # new values
        mpu.calibrate()
        mpu.configure()
        calib_data = np.array([mpu.readAccelerometerMaster() for ii in range(0,cal_size)])
        #
        ###################################
        # Plot with and without offsets
        ###################################
        #
        plt.style.use('ggplot')
        fig,axs = plt.subplots(2,1,figsize=(12,9))
        for ii in range(0,3):
            axs[0].plot(data[:,ii],
                        label='${}$, Uncalibrated'.format(accel_labels[ii]))
            axs[1].plot(calib_data[:,ii],
                        label='${}$, Calibrated'.format(accel_labels[ii]))
        axs[0].legend(fontsize=14);axs[1].legend(fontsize=14)
        axs[0].set_ylabel('$a_{x,y,z}$ [g]',fontsize=18)
        axs[1].set_ylabel('$a_{x,y,z}$ [g]',fontsize=18)
        axs[1].set_xlabel('Sample',fontsize=18)
        axs[0].set_ylim([-2,2]);axs[1].set_ylim([-2,2])
        axs[0].set_title('Accelerometer Calibration Calibration Correction',fontsize=18)
        fig.savefig('accel_calibration_output.png',dpi=300,
                    bbox_inches='tight',facecolor='#FCFCFC')
        fig.show()