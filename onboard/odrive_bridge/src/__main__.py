import lcm
import sys
import time as t
import odrive as odv
import threading
from rover_msgs import DriveStateCmd, DriveVelCmd, \
    DriveStateData, DriveVelData
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, \
    CTRL_MODE_VELOCITY_CONTROL, AXIS_STATE_FULL_CALIBRATION_SEQUENCE, \
    AXIS_STATE_IDLE, ENCODER_MODE_HALL

from odrive.utils import dump_errors


def main():
    global lcm_
    lcm_ = lcm.LCM()

    global modrive

    global legal_controller
    global legal_axis

    global vel_msg
    global state_msg

    global lock
    global speedlock

    legal_controller = int(sys.argv[1])
    legal_axis = sys.argv[2]

    vel_msg = DriveVelData()
    state_msg = DriveStateData()

    speedlock = threading.Lock()
    lock = threading.Lock()

    threading._start_new_thread(lcmThreaderMan, ())
    global odrive_bridge
    odrive_bridge = OdriveBridge()

    lock.acquire()
    odrive_bridge.on_event("disconnected odrive")
    """
    this sequence will start after odrive_bridge.run() is called
    looks for odrive --> goes to disarmed
    disarms --> goes to armed
    arms --> stay in armed
    """
    lock.release()

    while True:
        try:
            publish_state_msg(state_msg, str(self.state))
            odrive_bridge.run()
            
        except AttributeError:
            lock.acquire()
            odrive_bridge.on_event("disconnected odrive")
            lock.release()

    exit()


def lcmThreaderMan():
    lcm_1 = lcm.LCM()
    lcm_1.subscribe("/drive_state_cmd",
                    drive_state_cmd_callback)
    lcm_1.subscribe("/drive_vel_cmd", drive_vel)cmd_callback)
    while True:
        lcm_1.handle()

events = ["disconnected odrive", "disarm cmd", "arm cmd", "calibrate cmd", "odrive error"]
states = ["DisconnectedState", "DisarmedState", "ArmedState", "ErrorState"]
# Program states possible - BOOT,  DISARMED, ARMED, ERROR
# 							1		 2	      3	      4 

class State(object):
    """
    State object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self):
        print 'Processing current state:', str(self)

    def on_event(self, event):
        """
        Handle events that are delegated to this State.
        """
        pass

    def __repr__(self):
        """
        Make it so __str__ method can describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        State state
        str(state) = State
        """
        return self.__class__.__name__

class DisconnectedState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Disconnected State.
        """
        global modrive
        if (event == "arm cmd"):
            modrive.disarm()
            modrive.arm()
            return ArmedState()

        return self 

class DisarmedState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Disarmed State.
        """
        global modrive
        if (event == "disconnected odrive"):
            self.connect() 
            return DisconnectedState()        

        elif (event == "armed cmd"):
            modrive.arm()
            return ArmedState()

        elif (event == "calibrating cmd"):
            # sequence can be moved to armed ?
            modrive.calibrate()
            print ("done calibrating")
            modrive.disarm()
            return DisarmedState()

        elif (event == "odrive errors"):
            return ErrorState()

        return self

class ArmedState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Armed State.
        """
        global modrive

        if (event == "disarmed cmd"):
            modrive.disarm()
            return DisarmedState()

        elif (event == "disconnected odrive"):
            self.connect()
            return DisconnectedState()

        elif (event == "odrive errors"):
            return ErrorState()

        return self

class ErrorState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Error State.
        """
        global modrive
        print(dump_errors(modrive.odrive, True))
        if (event == "odrive errors"):
            try:
                modrive.reboot()  # only runs after initial pairing
            except:
                print('channel error caught')
            
            return DisconnectedState()

        return self 

class OdriveBridge(object): 

    def __init__(self):
        """ 
        Initialize the components. 
        Start with a Default State
        """
        self.state = DisarmedState()  # default is disarmed 
        self.encoder_time = 0

    def connect(self):
        global modrive
        global legal_controller
        print("looking for odrive")
        odrives = ["2091358E524B", "20563591524B"]
        id = odrive[legal_controller]

        print(id)
        odrive = odv.find_any(serial_number=id)

        print("found odrive")
        modrive = Modrive(odrive)  # arguments = odr
        modrive.set_current_lim(100)
        self.encoder_time = t.time()

    def on_event(self, event):
        """
        Incoming events are
        delegated to the given states which then handle the event. 
        The result is then assigned as the new state.
        The events we can send are disarm cmd, arm cmd, and calibrate cmd.
        """
        self.state = self.state.on_event(event)

    def run(self):
        if (self.state == DisarmedState()):
            if (t.time() - self.encoder_time > 0.1):  # order is flipped? why?
                self.encoder_time = publish_encoder_msg(vel_msg)

        elif (self.state == ArmedState()):
            global speedlock
            global left_speed
            global right_speed

            if (self.encoder_time - t.time() > 0.1):
                self.encoder_time = publish_encoder_msg(vel_msg)

            speedlock.acquire()
            modrive.set_vel("LEFT", left_speed)
            modrive.set_vel("RIGHT", right_speed)
            speedlock.release()

        elif (self.state == DisconnectedState()):
            lock.acquire()
            self.on_event("arm cmd")
            lock.release()

        errors = modrive.check_errors()

        if errors:
            lock.acquire()
            self.on_event("odrive error")
            lock.release()
            # first time will set to ErrorState
            # second time will reboot

    def get_state():
        return str(self.state)

""" 
call backs
"""

def publish_state_msg(msg, state):
    global legal_controller
    msg.state = states.index(state)
    msg.controller = legal_controller
    lcm_.publish("/drive_state_data", msg.encode())
    print("changed state to " + state)

def publish_encoder_helper(msg, axis):
    global modrive
    global legal_controller
    msg.measuredCurrent = modrive.get_iq_measured(axis)
    msg.estimatedVel = modrive.get_vel_estimate(axis)

    motor_map = {("LEFT", 0): 0, ("RIGHT", 0): 1,
                 ("LEFT", 1): 2, ("RIGHT", 1): 3}
    msg.axis = motor_map[(axis, legal_controller)]

    lcm_.publish("/drive_vel_data", msg.encode())


def publish_encoder_msg(msg):
    publish_encoder_helper(msg, "LEFT")
    publish_encoder_helper(msg, "RIGHT")
    return t.time()


def drive_state_cmd_callback(channel, msg):
    print("requested state call back is being called")
    global odrive_bridge
    global legal_controller

    command_list = ["disarm cmd", "armd cmd", "calibrating cmd"]
    lock.acquire()
    cmd = DriveStateCmd.decode(msg)
    if cmd.controller == legal_controller:  # Check which controller
        odrive_bridge.on_event(command_list[cmd.state - 1])
    lock.release()


def drive_vel_cmd_callback(channel, msg):
    # set the odrive's velocity to the float specified in the message
    # no state change

    global left_speed
    global right_speed
    global speedlock

    cmd = DriveVelCmd.decode(msg)

    speedlock.acquire()
    left_speed = cmd.left
    right_speed = cmd.right
    speedlock.release()

if __name__ == "__main__":
    main()


class Modrive:
    CURRENT_LIM = 30

    def __init__(self, odr):
        self.odrive = odr
        self.front_axis = self.odrive.axis0
        self.back_axis = self.odrive.axis1
        self.set_current_lim(self.CURRENT_LIM)

    # viable to set initial state to idle?

    def __getattr__(self, attr):
        if attr in self.__dict__:
            return getattr(self, attr)
        return getattr(self.odrive, attr)

    def calibrate(self):
        self._reset(self.front_axis)
        self._reset(self.back_axis)
        self.odrive.save_configuration()
        # the guide says to reboot here...

        self._requested_state("LEFT", AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        while (self.get_current_state("RIGHT") != AXIS_STATE_IDLE):
            pass
        self._requested_state("LEFT", AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        while (self.get_current_state("RIGHT") != AXIS_STATE_IDLE):
            pass
        
        front_state, back_state = self.get_current_state()

        # if both are idle it means its done calibrating
        if front_state == AXIS_STATE_IDLE \
                and back_state == AXIS_STATE_IDLE:
            self._pre_calibrate(self.front_axis)
            self._pre_calibrate(self.back_axis)
            self.odrive.save_configuration()
        # also says to reboot here...

    def disarm(self):
        self.set_current_lim(100)
        self.closed_loop_ctrl()
        self.set_velocity_ctrl()

        self.set_vel("LEFT", 0)
        self.set_vel("RIGHT", 0)

        self.idle()

    def arm(self):
        self.closed_loop_ctrl()
        self.set_velocity_crtl()

    def set_current_lim(self, lim):
        self.front_axis.motor.config.current_lim = lim
        self.back_axis.motor.config.current_lim = lim

    def _set_control_mode(self, mode):
        self.front_axis.controller.config.control_mode = mode
        self.back_axis.controller.config.control_mode = mode

    def set_velocity_ctrl(self):
        self._set_control_mode(CTRL_MODE_VELOCITY_CONTROL)

    def get_iq_measured(self, axis):
        if (axis == "LEFT"):
            return self.front_axis.motor.current_control.Iq_measured
        elif(axis == "RIGHT"):
            return self.back_axis.motor.current_control.Iq_measured

    def get_vel_estimate(self, axis):
        # axis = self.odrive[axis_number]
        if (axis == "LEFT"):
            return self.front_axis.encoder.vel_estimate
        elif(axis == "RIGHT"):
            return self.back_axis.encoder.vel_estimate

    def idle(self):
        self._requested_state(AXIS_STATE_IDLE)

    def closed_loop_ctrl(self):
        self._requested_state(AXIS_STATE_CLOSED_LOOP_CONTROL)

    def _requested_state(self, axis, state):
        self.back_axis.requested_state = state
        self.front_axis.requested_state = state

    def set_vel(self, axis, vel):
        if (axis == "LEFT"):
            self.front_axis.controller.vel_setpoint = vel
        elif axis == "RIGHT":
            self.back_axis.controller.vel_setpoint = vel

    def get_current_state(self):
        return (self.front_axis.current_state, self.back_axis.current_state)

    def _reset(self, m_axis):
        m_axis.motor.config.pole_pairs = 15
        m_axis.motor.config.resistance_calib_max_voltage = 4
        m_axis.motor.config.requested_current_range = 25
        m_axis.motor.config.current_control_bandwidth = 100

        m_axis.encoder.config.mode = ENCODER_MODE_HALL
        m_axis.encoder.config.cpr = 90
        m_axis.encoder.config.bandwidth = 100
        m_axis.controller.config.pos_gain = 1
        m_axis.controller.config.vel_gain = 0.02
        m_axis.controller.config.vel_integrator_gain = 0.1
        m_axis.controller.config.vel_limit = 1000
        m_axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

    def _pre_calibrate(self, m_axis):
        self.m_axis.motor.config.pre_calibrated = True
        self.m_axis.encoder.config.pre_calibrated = True

    def check_errors(self):
        front = self.front_axis.error
        back = self.back_axis.error
        return back + front
