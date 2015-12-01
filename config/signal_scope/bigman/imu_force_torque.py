import numpy
# accel plot
addPlot(timeWindow=30, yLimits=[-10, 10])
addSignal('MICROSTRAIN_INS',msg.utime, msg.accel[0])
addSignal('MICROSTRAIN_INS',msg.utime, msg.accel[1])
addSignal('MICROSTRAIN_INS',msg.utime, msg.accel[2])
def accelNormFunction(msg):
    '''accel norm'''
    return msg.utime, numpy.linalg.norm(msg.accel)
addSignalFunction('MICROSTRAIN_INS', accelNormFunction)

addSignal('ATLAS_IMU_BATCH',msg.utime, msg.raw_imu[0].linear_acceleration[0])
addSignal('ATLAS_IMU_BATCH',msg.utime, msg.raw_imu[0].linear_acceleration[1])
addSignal('ATLAS_IMU_BATCH',msg.utime, msg.raw_imu[0].linear_acceleration[2])

def accelNormFunction2(msg):
    '''accel norm'''
    return msg.utime, numpy.linalg.norm(msg.raw_imu[0].linear_acceleration)
addSignalFunction('ATLAS_IMU_BATCH', accelNormFunction2)

# gyro plot
addPlot(timeWindow=30, yLimits=[-2, 2])
addSignal('MICROSTRAIN_INS',msg.utime, msg.gyro[0])
addSignal('MICROSTRAIN_INS',msg.utime, msg.gyro[1])
addSignal('MICROSTRAIN_INS',msg.utime, msg.gyro[2])

def rot0Function(msg):
    '''rot0'''
    return msg.utime, msg.raw_imu[0].delta_rotation[0]*1000.0
def rot1Function(msg):
    '''rot1'''
    return msg.utime, msg.raw_imu[0].delta_rotation[1]*1000.0
def rot2Function(msg):
    '''rot2'''
    return msg.utime, msg.raw_imu[0].delta_rotation[2]*1000.0

addSignalFunction('ATLAS_IMU_BATCH', rot0Function)
addSignalFunction('ATLAS_IMU_BATCH', rot1Function)
addSignalFunction('ATLAS_IMU_BATCH', rot2Function)


# ft plot
addPlot(timeWindow=30, yLimits=[0, 1600])
addSignal('FORCE_TORQUE',msg.utime, msg.sensors[0].force[2])
addSignal('FORCE_TORQUE',msg.utime, msg.sensors[1].force[2])

addSignal('ATLAS_STATE',msg.utime, msg.force_torque.l_foot_force_z)
addSignal('ATLAS_STATE',msg.utime, msg.force_torque.r_foot_force_z)
