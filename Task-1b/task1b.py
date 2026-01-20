import math
import sys

# ---------- Global Container ----------
S = {}

# State indices
TILT_ANGLE = 0
TILT_VEL = 1
WHEEL_POS_ERROR = 2
WHEEL_VEL = 3

def sysCall_init():
    """
    Initializes scene object handles and control parameters.
    """
    global S
    sim = require('sim')
    S['sim'] = sim

    # --- 1. Get Object Handles ---
    S['body_h'] = sim.getObject('/body')
    S['rjoint_h'] = sim.getObject('/right_joint')
    S['ljoint_h'] = sim.getObject('/left_joint')

    # --- 2. LQR Gains ---
    S['K'] = [
        -120.0,  # Tilt Angle
        -50.0,   # Tilt Velocity
        -5.0,    # Wheel Position Error
        -15.0    # Wheel Velocity
    ]

    # --- Yaw Control ---
    S['Kyaw'] = 0.0
    S['desired_yaw'] = 0.0
    S['current_yaw'] = 0.0

    # --- Joint Parameters ---
    # REMOVED the two forbidden sim.setJointTargetForce lines.
    sim.setJointTargetVelocity(S['rjoint_h'], 0)
    sim.setJointTargetVelocity(S['ljoint_h'], 0)

    # --- State Variables ---
    S['desired_wheel_pos'] = 0.0
    S['x'] = [0.0] * 4
    S['desired_velocity'] = 0.1

def sysCall_sensing():
    """
    Reads sensor data and computes state error vector x.
    """
    sim = S['sim']
    dt = sim.getSimulationTimeStep()

    # --- Body orientation & velocity ---
    orientation = sim.getObjectOrientation(S['body_h'], sim.handle_world)
    tilt_angle = orientation[0]
    _, angular_velocity = sim.getObjectVelocity(S['body_h'], sim.handle_world)
    tilt_vel = angular_velocity[0]

    # --- Yaw sensing ---
    S['current_yaw'] = orientation[2]

    # --- Wheel joint sensing ---
    wheel_pos = sim.getJointPosition(S['rjoint_h'])
    wheel_vel = sim.getJointVelocity(S['rjoint_h'])

    # --- Update desired wheel position ---
    S['desired_wheel_pos'] += S['desired_velocity'] * dt

    # --- State error vector ---
    S['x'][TILT_ANGLE] = -tilt_angle
    S['x'][TILT_VEL] = -tilt_vel
    S['x'][WHEEL_POS_ERROR] = wheel_pos - S['desired_wheel_pos']
    S['x'][WHEEL_VEL] = wheel_vel

def sysCall_actuation():
    """
    Calculates control input and applies it to the wheels.
    """
    sim = S['sim']
    x = S['x']

    # --- LQR Control Law ---
    U = -(S['K'][TILT_ANGLE] * x[TILT_ANGLE] +
          S['K'][TILT_VEL] * x[TILT_VEL] +
          S['K'][WHEEL_POS_ERROR] * x[WHEEL_POS_ERROR] +
          S['K'][WHEEL_VEL] * x[WHEEL_VEL])

    # --- Clamp control ---
    max_velocity = 10.0
    U = max(-max_velocity, min(max_velocity, U))

    # --- Yaw correction ---
    yaw_error = S['desired_yaw'] - S['current_yaw']
    yaw_correction = S['Kyaw'] * yaw_error

    # --- Apply velocities ---
    sim.setJointTargetVelocity(S['rjoint_h'], U - yaw_correction)
    sim.setJointTargetVelocity(S['ljoint_h'], U + yaw_correction)

def sysCall_cleanup():
    pass
