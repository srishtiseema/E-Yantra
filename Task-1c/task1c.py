import math
import sys
import time

# ---------- Global Container ----------
S = {}

# State indices
TILT_ANGLE = 0
TILT_VEL = 1
WHEEL_POS_ERROR = 2
WHEEL_VEL = 3

def sysCall_init():
    global S
    sim = require('sim')
    S['sim'] = sim

    # --- Get Object Handles ---
    S['body_h'] = sim.getObject('/body')
    S['rjoint_h'] = sim.getObject('/right_joint')
    S['ljoint_h'] = sim.getObject('/left_joint')

    # --- LQR Gains (Your Proven Values) ---
    S['K'] = [
        -110.0,  # Tilt Angle
        -40.0,   # Tilt Velocity
        -0.8,    # Wheel Position Error
        -10.0    # Wheel Velocity
    ]

    # --- Turning Gain ---
    S['Kyaw'] = 1.2

    # --- Initialize control vars ---
    S['desired_wheel_pos'] = 0.0
    S['desired_velocity'] = 0.0
    S['turn_input'] = 0.0
    S['x'] = [0.0] * 4

    # Key handling system
    S['keys'] = {'up': 0, 'down': 0, 'left': 0, 'right': 0}
    S['last_key_time'] = 0.0
    S['key_timeout'] = 0.2

    sim.setJointTargetVelocity(S['rjoint_h'], 0)
    sim.setJointTargetVelocity(S['ljoint_h'], 0)
    sim.addLog(sim.verbosity_scriptinfos, "Child script init done: manual arrow control enabled")

def sysCall_sensing():
    sim = S['sim']
    dt = sim.getSimulationTimeStep()

    # --- Sensor Reading ---
    orientation = sim.getObjectOrientation(S['body_h'], sim.handle_world)
    tilt_angle = orientation[0]
    _, angular_velocity = sim.getObjectVelocity(S['body_h'], sim.handle_world)
    tilt_vel = angular_velocity[0]
    wheel_pos = sim.getJointPosition(S['rjoint_h'])
    wheel_vel = sim.getJointVelocity(S['rjoint_h'])

    # --- Keyboard Input Logic (Unchanged) ---
    message, data, _ = sim.getSimulatorMessage()
    if message == sim.message_keypress:
        key = data[0]
        S['last_key_time'] = sim.getSimulationTime()
        for k in S['keys']: S['keys'][k] = 0
        if key == 2007: S['keys']['up'] = 1
        elif key == 2008: S['keys']['down'] = 1
        elif key == 2010: S['keys']['left'] = 1
        elif key == 2009: S['keys']['right'] = 1

    if sim.getSimulationTime() - S['last_key_time'] > S['key_timeout']:
        for k in S['keys']: S['keys'][k] = 0

    if S['keys']['up']: S['desired_velocity'] = 0.25
    elif S['keys']['down']: S['desired_velocity'] = -0.25
    else: S['desired_velocity'] = 0.0

    if S['keys']['left']: S['turn_input'] = -1.0
    elif S['keys']['right']: S['turn_input'] = 1.0
    else: S['turn_input'] = 0.0

    # --- DEFINITIVE FIX: Unified State Update ---
    # 1. The target position ONLY moves when you command it with the keyboard.
    #    When desired_velocity is 0, the target stands still, creating a fixed anchor.
    S['desired_wheel_pos'] += S['desired_velocity'] * dt

    # 2. The errors are now calculated in a simple, consistent way.
    pos_error_term = wheel_pos - S['desired_wheel_pos']
    # The velocity error is the difference between actual speed and your commanded speed.
    vel_state_term = wheel_vel - S['desired_velocity']

    # --- Prepare state vector ---
    S['x'][TILT_ANGLE] = -tilt_angle
    S['x'][TILT_VEL] = -tilt_vel
    S['x'][WHEEL_POS_ERROR] = pos_error_term
    S['x'][WHEEL_VEL] = vel_state_term

    sim.addLog(sim.verbosity_scriptinfos,
               f"tilt={tilt_angle:+.3f} vel={S['desired_velocity']:+.2f} turn={S['turn_input']:+.1f}")

def sysCall_actuation():
    sim = S['sim']
    x = S['x']

    # --- Base LQR Control ---
    U = -(S['K'][TILT_ANGLE] * x[TILT_ANGLE] +
          S['K'][TILT_VEL] * x[TILT_VEL] +
          S['K'][WHEEL_POS_ERROR] * x[WHEEL_POS_ERROR] +
          S['K'][WHEEL_VEL] * x[WHEEL_VEL])
          
    # --- Add the desired velocity as a "feedforward" term ---
    # This tells the bot "Your baseline speed should be the one I'm commanding"
    U += S['desired_velocity']

    # --- Clamp motor command ---
    max_vel = 7.0
    U = max(-max_vel, min(max_vel, U))

    # --- Add manual yaw correction ---
    yaw_correction = S['Kyaw'] * S['turn_input']

    # --- Apply motor commands ---
    sim.setJointTargetVelocity(S['rjoint_h'], U - yaw_correction)
    sim.setJointTargetVelocity(S['ljoint_h'], U + yaw_correction)

def sysCall_cleanup():
    pass
