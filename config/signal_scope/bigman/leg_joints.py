right_joints = ['RHipLat', 'RHipYaw', 'RHipSag', 'RKneeSag', 'RAnkSag', 'RAnkLat']
left_joints = ['LHipLat', 'LHipYaw', 'LHipSag', 'LKneeSag', 'LAnkSag', 'LAnkLat']
names = msg.joint_name


addPlot()
addSignals('CORE_ROBOT_STATE', msg.utime, msg.joint_position, right_joints, keyLookup=names)

addPlot()
addSignals('CORE_ROBOT_STATE', msg.utime, msg.joint_position, left_joints, keyLookup=names)
