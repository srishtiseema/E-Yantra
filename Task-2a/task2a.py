import math
import sys

# PID helper function: computes proportional, integral, and derivative contributions
def calc_pid(kp,ki,kd,error,prev_error,integ_sum):
    # Proportional response (reacts to current error)
    p_term=kp*error

    # Integral buildup (helps eliminate small steady-state error)
    integ_sum+=error*0.5
    # Prevent integral wind-up by clamping it
    integ_sum=max(-10.0,min(10.0,integ_sum))
    i_term=ki*integ_sum

    # Derivative response (reacts to sudden change)
    d_term=kd*((error-prev_error)/0.5)

    # Return combined PID output and updated integral sum
    return p_term+i_term+d_term,integ_sum


# Called once at simulation start: initialize everything
def sysCall_init():
    sim=require('sim')
    self.sim=sim

    # Look up handles for robot parts
    self.obj_body=sim.getObject('/body')
    self.j_left=sim.getObject('/left_joint')
    self.j_right=sim.getObject('/right_joint')
    self.j_arm=sim.getObject('/arm_joint')
    self.j_grip=sim.getObject('/Prismatic_joint')

    # PID gains for tilt (balance) and forward position control
    self.pid_tilt=(15,2.0,0.2)
    self.pid_pos=(50,0.6,0.6)

    # Error tracking for both PIDs
    self.prev_tilt_err=0.0
    self.prev_pos_err=0.0
    self.int_tilt=0.0
    self.int_pos=0.0

    # Measured robot state values
    self.pitch_now=0.0
    self.pos_now=0.0
    self.pos_target=0.0

    # User inputs (steering, arm, gripper)
    self.inp_turn=0.0
    self.inp_arm=0.0
    self.inp_grip=0.0

    # Motion speed scaling values
    self.turn_rate=8.0
    self.arm_rate=5.0
    self.grip_rate=0.7

    # Limits for smooth incremental position updates
    self.position_step=0.1
    self.last_target_val=0.0

    # Distances at which robot slows down & stops
    self.dec_thresh=0.15
    self.stop_thresh=0.02


# Runs every simulation step → computes wheel, arm, gripper commands
def sysCall_actuation():
    sim=self.sim

    # Tilt error (balance target is perfectly upright = 0 pitch)
    tilt_error=0-self.pitch_now

    # Position error (distance robot still needs to travel)
    pos_error=self.pos_target-self.pos_now
    abs_dist=abs(pos_error)

    # If extremely close to target, damp integral to prevent oscillation
    if abs_dist<self.stop_thresh:
        self.int_pos*=0.5

    # Read current PID gain values
    tk,ti,td=self.pid_tilt
    pk,pi,pd=self.pid_pos

    # As robot approaches destination, gradually reduce gains for smooth stopping
    if abs_dist<self.dec_thresh:
        factor=abs_dist/self.dec_thresh
        pk*=(0.3+0.7*factor)
        pd*=(0.5+0.5*factor)

    # Compute outputs of both PID controllers
    tilt_out,self.int_tilt=calc_pid(tk,ti,td,tilt_error,self.prev_tilt_err,self.int_tilt)
    pos_out,self.int_pos=calc_pid(pk,pi,pd,pos_error,self.prev_pos_err,self.int_pos)

    # Final wheel base command = keep balance - hold position
    base_cmd=tilt_out-pos_out

    # Steering contribution (from user)
    steer_cmd=self.inp_turn*self.turn_rate

    # Convert control outputs to left & right wheel speeds
    left_v=(base_cmd*10)+steer_cmd
    right_v=(base_cmd*10)-steer_cmd
    sim.setJointTargetVelocity(self.j_left,left_v)
    sim.setJointTargetVelocity(self.j_right,right_v)

    # Move arm and gripper with scaled user inputs
    sim.setJointTargetVelocity(self.j_arm,self.inp_arm*self.arm_rate)
    sim.setJointTargetVelocity(self.j_grip,self.inp_grip*self.grip_rate)

    # Store last errors for next derivative term
    self.prev_tilt_err=tilt_error
    self.prev_pos_err=pos_error


# Reads robot sensors, orientation, and handles keyboard input
def sysCall_sensing():
    sim=self.sim

    # Read current Y-position (robot moves in Y direction)
    pos=sim.getObjectPosition(self.obj_body,-1)
    self.pos_now=pos[1]

    # Read robot’s pitch angle
    ori=sim.getObjectOrientation(self.obj_body,-1)
    _,_,r=sim.alphaBetaGammaToYawPitchRoll(*ori)
    self.pitch_now=r

    # Default inputs to zero (prevents stuck commands)
    self.inp_turn=0.0
    self.inp_arm=0.0
    self.inp_grip=0.0
    movement_flag=False

    # Check for keyboard events
    msg,data,_=sim.getSimulatorMessage()

    if msg==sim.message_keypress:
        key=data[0]

        # UP arrow → move target slightly forward
        if key==2007:
            nxt=self.pos_target+0.01
            if abs(nxt-self.last_target_val)<self.position_step:
                self.pos_target=nxt
            movement_flag=True

        # DOWN arrow → move target backward
        elif key==2008:
            nxt=self.pos_target-0.01
            if abs(nxt-self.last_target_val)<self.position_step:
                self.pos_target=nxt
            movement_flag=True

        # Left/right arrows → turning
        elif key==2009:
            self.inp_turn=1.0
        elif key==2010:
            self.inp_turn=-1.0

        # Q/E → gripper open/close
        elif key==113:
            self.inp_grip=-1.0
        elif key==101:
            self.inp_grip=1.0

        # W/S → raise/lower arm
        elif key==119:
            self.inp_arm=-1.0
        elif key==115:
            self.inp_arm=1.0

    # If no active movement key, slowly fade position integral to avoid drift
    if not movement_flag:
        self.int_pos*=0.9

    # Store last target (used to limit per-step movement)
    self.last_target_val=self.pos_target


def sysCall_cleanup():
    pass
