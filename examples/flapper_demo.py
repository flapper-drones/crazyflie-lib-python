# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2019 Bitcraze AB
#  Copyright (C) 2021 Flapper Drones
#
#  Flapper Drone Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to one Flapper, sets the initial position/yaw
and flies a trajectory.

The initial pose (x, y, z, yaw) is configured as 'origin' and 'initial_yaw'
The trajectory is defined with respect to the origin.
"""
import math
import sys
import time
import csv
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from datetime import datetime

import threading as th

kill_flight = False
def key_capture_thread():
    global kill_flight
    input()
    kill_flight = True

address = 'E7E7E7E7E7' # NimbleFlapper 2.0 Bolt

# URI to the drone to connect to
uri = 'radio://0/80/2M/' + address

POSITION_X = []
POSITION_Y = []
POSITION_Z = []
TARGET_X = []
TARGET_Y = []
TARGET_Z = []
VX = []
VY = []
VZ = []
TARGET_VX = []
TARGET_VY = []
TARGET_VZ = []
ROLL = []
PITCH = []
YAW = []
CMD_ROLL = []
CMD_PITCH = []
CMD_YAW = []
# Change the sequence according to your setup
#             x    y    z

a = 1 #length of side
h = 1 #height

# Initial position
origin = [0.0, 0.0, 0.0]

initial_yaw = 0.0  # In degrees
# 0: positive X direction
# 90: positive Y direction
# 180: negative X direction
# 270: negative Y direction

# Sequence relative to the origin!
sequence = [
    #hover & step forward
    (0, 0, 0.5*h),
    (0, 0, h),
    (a, 0, h),
    (a, 0, h),
    (0, 0, h),
    (0, 0, 0.5*h),
    (0, 0, 0),
]


yaw_radians = math.radians(initial_yaw)

def wait_for_position_estimator(scf):
    print('Waiting for estimator to find position...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001  #0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            # print("{} {} {}".
            #       format(max_x - min_x, max_y - min_y, max_z - min_z))

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def set_control_parameters_Bolt(scf):
    print('Setting control parameters for Nimble Flapper Bolt')

    # Example: 
    # scf.cf.param.set_value('attFilt.rateFiltEn', '1') #1

def set_initial_position(scf, x, y, z, yaw):
    scf.cf.param.set_value('kalman.initialX', x)
    scf.cf.param.set_value('kalman.initialY', y)
    scf.cf.param.set_value('kalman.initialZ', z)
    scf.cf.param.set_value('kalman.initialYaw', yaw_radians)


def reset_tumble_detector(scf):
    cf = scf.cf
    cf.param.set_value('stabilizer.stop', '0')
    time.sleep(0.1)

def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

    wait_for_position_estimator(cf)

def reinitialize_controller(scf):
    cf = scf.cf
    cf.param.set_value('stabilizer.controller', '0') # switch to ANY
    time.sleep(0.1)
    cf.param.set_value('stabilizer.controller', '1') # switch to PID
    time.sleep(0.1)

def log_async_setup(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_async_callback)

def log_and_print_async_setup(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_and_print_async_callback)

def log_async_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    if 'pm.vbat' in data:
        global vbat
        vbat = data['pm.vbat']

def log_and_print_async_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    print('[%d]: %s' % (timestamp, data))


def position_estimate_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    position_x = data["stateEstimate.x"]
    position_y = data["stateEstimate.y"]
    position_z = data["stateEstimate.z"]
    POSITION_X.append(position_x)
    POSITION_Y.append(position_y)
    POSITION_Z.append(position_z)

def velocity_estimate_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    velocity_x = data["stateEstimate.vx"]
    velocity_y = data["stateEstimate.vy"]
    velocity_z = data["stateEstimate.vz"]
    VX.append(velocity_x)
    VY.append(velocity_y)
    VZ.append(velocity_z)

def attitude_estimate_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    roll = data["stateEstimate.roll"]
    pitch = data["stateEstimate.pitch"]
    yaw = data["stateEstimate.yaw"]
    ROLL.append(roll)
    PITCH.append(pitch)
    YAW.append(yaw)

def position_control_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    tar_x = data["posCtl.targetX"]
    tar_y = data["posCtl.targetY"]
    tar_z = data["posCtl.targetZ"]
    TARGET_X.append(tar_x)
    TARGET_Y.append(tar_y)
    TARGET_Z.append(tar_z)

def velocity_control_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    tar_vx = data["posCtl.targetVX"]
    tar_vy = data["posCtl.targetVY"]
    tar_vz = data["posCtl.targetVZ"]
    TARGET_VX.append(tar_vx)
    TARGET_VY.append(tar_vy)
    TARGET_VZ.append(tar_vz)

def attitude_control_callback(timestamp, data, logconf):
    print('[%d]: %s' % (timestamp, data), file=f)
    roll = data["controller.roll"]
    pitch = data["controller.pitch"]
    yaw = data["controller.yaw"]
    CMD_ROLL.append(roll)
    CMD_PITCH.append(pitch)
    CMD_YAW.append(yaw)

def activate_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # define logging parameters

    log_vbat = LogConfig(name='pm', period_in_ms=500)
    log_vbat.add_variable('pm.vbat', 'float')

    log_pos_estimate = LogConfig(name='stateEstimate', period_in_ms=50)
    log_pos_estimate.add_variable('stateEstimate.x', 'float')
    log_pos_estimate.add_variable('stateEstimate.y', 'float')
    log_pos_estimate.add_variable('stateEstimate.z', 'float')

    log_att_estimate = LogConfig(name='stateEstimate', period_in_ms=50)
    log_att_estimate.add_variable('stateEstimate.roll', 'float')
    log_att_estimate.add_variable('stateEstimate.pitch', 'float')
    log_att_estimate.add_variable('stateEstimate.yaw', 'float')

    log_vel_estimate = LogConfig(name='stateEstimate', period_in_ms=50)
    log_vel_estimate.add_variable('stateEstimate.vx', 'float')
    log_vel_estimate.add_variable('stateEstimate.vy', 'float')
    log_vel_estimate.add_variable('stateEstimate.vz', 'float')

    log_pos_ctrl = LogConfig(name='posCtl', period_in_ms=50)
    log_pos_ctrl.add_variable('posCtl.targetX', 'float')
    log_pos_ctrl.add_variable('posCtl.targetY', 'float')
    log_pos_ctrl.add_variable('posCtl.targetZ', 'float')

    log_att_ctrl = LogConfig(name='controller', period_in_ms=50)
    log_att_ctrl.add_variable('controller.roll', 'float') # these are setpoints!!!
    log_att_ctrl.add_variable('controller.pitch', 'float')
    log_att_ctrl.add_variable('controller.yaw', 'float')

    log_vel_ctrl = LogConfig(name='posCtl', period_in_ms=50)
    log_vel_ctrl.add_variable('posCtl.targetVX', 'float')
    log_vel_ctrl.add_variable('posCtl.targetVY', 'float')
    log_vel_ctrl.add_variable('posCtl.targetVZ', 'float')


    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with open(datetime.now().strftime("%Y%m%d-%H%M%S") + '.txt', 'w') as f:
            print('Flight log' + datetime.now().strftime("%Y%m%d %H%M%S"), file=f)

            scf.cf.log.add_config(log_pos_estimate)
            log_pos_estimate.data_received_cb.add_callback(position_estimate_callback)
            scf.cf.log.add_config(log_vel_estimate)
            log_vel_estimate.data_received_cb.add_callback(velocity_estimate_callback)
            scf.cf.log.add_config(log_att_estimate)
            log_att_estimate.data_received_cb.add_callback(attitude_estimate_callback)
            scf.cf.log.add_config(log_pos_ctrl)
            log_pos_ctrl.data_received_cb.add_callback(position_control_callback)
            scf.cf.log.add_config(log_vel_ctrl)
            log_vel_ctrl.data_received_cb.add_callback(velocity_control_callback)
            scf.cf.log.add_config(log_att_ctrl)
            log_att_ctrl.data_received_cb.add_callback(attitude_control_callback)


            log_async_setup(scf, log_vbat)
            log_vbat.start()

            cf = scf.cf

            time.sleep(1)
            print('Battery voltage is {}' .format(vbat))

            reset_tumble_detector(scf)

            set_control_parameters_Bolt(scf)
            
            # reinitialize_controller(scf) # to load all new control parameters

            set_initial_position(scf, origin[0], origin[1], origin[2], yaw_radians)
            reset_estimator(scf)

            log_pos_estimate.start()
            log_vel_estimate.start()
            log_att_estimate.start()
            log_pos_ctrl.start()
            log_vel_ctrl.start()
            log_att_ctrl.start()

            activate_high_level_commander(cf)
            time.sleep(0.2)
            commander = cf.high_level_commander

            commander.takeoff(-0.5, 0.001) # setting the setpoint underground to spinup the motors before taking off //TODO improve the startup sequence using motor arming?
            time.sleep(0.01)
            commander.go_to(origin[0], origin[1], origin[2]-0.5, yaw_radians, 0.01) # (re)set the setpoint, sometimes xy stays at [0,0]
            time.sleep(0.2)
            commander.go_to(origin[0], origin[1], origin[2], yaw_radians, 1.5)
            time.sleep(0.2)

            for position in sequence:
                print('Setting position {}'.format(position))

                x = position[0] + origin[0]
                y = position[1] + origin[1]
                z = position[2] + origin[2]

                commander.go_to(x, y, z, yaw_radians, 4)
                time.sleep(4)

            commander.land(origin[2] - 0.5, 3.0, yaw_radians)
            time.sleep(3.0)
            commander.stop()

            log_vbat.stop()
            log_pos_estimate.stop()
            log_vel_estimate.stop()
            log_att_estimate.stop()
            log_pos_ctrl.stop()
            log_vel_ctrl.stop()
            log_att_ctrl.stop()

