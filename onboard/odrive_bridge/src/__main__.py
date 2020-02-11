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


def main():
    global lcm_
    lcm_ = lcm.LCM()

    threading._start_new_thread(lcmThreaderMan, ())
    OdriveBridge odrive_bridge;

    global event
    event = "BOOT"

    while True:
        try:
            event = odrive_bridge.on_event(event)
        except AttributeError:
            print ("odrive disconnected")
            event = "BOOT"
        t.sleep(1)

    exit()


def lcmThreaderMan():
    lcm_1 = lcm.LCM()
    lcm_1.subscribe("/drivestatecmd",
                    drivestatecmd_callback)
    lcm_1.subscribe("/drivevelcmd", drivevelcmd_callback)
    while True:
        lcm_1.handle()
        t.sleep(1)


states = ["BOOT", "DISARMED", "ARMED", "ERROR", "CALIBRATING"]
# Program states possible - BOOT,  DISARMED, ARMED, ERROR, CALIBRATING
# 							1		 2	      3	      4       5
class State(object):
    """
    We define a state object which provides some utility functions for the
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
        Leverages the __str__ method to describe the State.
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        """
        return self.__class__.__name__


class DisconnectedState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Disconnected State.
        """
        if (event == "BOOT"):  # always goes from boot to disarm
            return DisarmedState()
        return self

class DisarmedState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Disarmed State.
        """
        if (event == "BOOT"):
            return DisconnectedState()

        elif (event == "ARMED"):
            return ArmedState()

        elif (event == "CALIBRATING"):
            return CalibratingState()

        elif (event == "ERROR"):
            return ErrorState()

        return self

class ArmedState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Armed State.
        """
        if (event == "DISARMED"):
            return DisarmedState()

        elif (event == "ERROR"):
            return ErrorState()

        return self

class CalibratingState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Calibrating State.
        """
        if (event == "CALIBRATING"):
            return DisarmedState()

        elif (event == "ERROR"):
            return ErrorState()

        return self

class ErrorState(State):
    def on_event(self, event):
        """
        Handle events that are delegated to the Error State.
        """
        if (event == "ERROR"):  # always tries to calibrate when it errors out
            return ErrorState()

        return self

class OdriveBridge(object): #object = odrive 

    state_to_event = {
        "DisconnectedState" : "BOOT", 
        "DisarmedState" : "DISARM",
        "ArmedState" : "ARMED",
        "ErrorState" : "ERROR",
        "CalibratingState" : "CALIBRATING"
        }

    def __init__(self):
        """ Initialize the components. """
        global odrive = None  # starting odrive
        global modrive

        global legalController
        global legalAxis

        legalController = int(sys.argv[1])
        legalAxis = sys.argv[2]
        if (legalAxis != "FRONT" and legalAxis != "BACK" and legalAxis != "BOTH"):
            print("invalid odrive axis given")

        global msg
        global msg1
        msg = DriveVelData()
        msg1 = DriveStateData()

        global lock
        lock = threading.Lock()

        global encoderTime
        encoderTime = 0

        # Start with a default state.
        self.state = DisconnectedState()

    def on_event(self, event):
        """
        Incoming events are
        delegated to the given states which then handle the event. The result is
        then assigned as the new state.
        """
        # events we can give are disarmed, armed, and calibrating
        global lock
        lock.acquire()

        publish_state_msg(msg1, state_to_event[str(self.state)])
        # publishes current state before events

        errors = modrive.check_errors()
        if errors:
            # sets state to error
            self.state = ErrorState()
            event = "ERROR"

        self.state = self.state.on_event(event)
        # gets the state that the odrive should be switched to 

        if (self.state == DisconnectedState()):
            try:
                odrive.reboot()  # only runs after initial pairing
            except:
                print('channel error caught')
            print("looking for odrive")

            if sys.argv[1] == "0":
                id = "2091358E524B"
            if sys.argv[1] == "1":
                id = "20563591524B"
            print(id)

            odrive = odv.find_any(serial_number=id)
            print("found odrive")

            modrive = Modrive(odrive)  # arguments = odr
            modrive.reset()
            modrive.set_current_lim(100)
            modrive.set_velocity_ctrl()

            # sets current state to disarmed
            encoderTime = t.time()
            # The next state will be the result of the on_event function.

        elif (self.state == DisarmedState()):
            if (t.time() - encoderTime > 0.1):
                print("Sent Encoder Message")
                encoderTime = t.time()
                publish_encoder_msg(msg)

            modrive.closed_loop_ctrl()
            modrive.set_velocity_ctrl()

            if (modrive.get_vel_estimate("FRONT") != 0 \
                      or modrive.get_vel_estimate("BACK") != 0):
                modrive.set_vel(legalAxis, 0)
            modrive.idle()

        elif (self.state == ArmedState()):
            modrive.closed_loop_ctrl()
            modrive.set_velocity_crtl() # can only get to armed from disarmed !!! TODO: maybe fix 

            if (encoderTime - t.time() > 0.1):
                print("Sent Encoder Message")
                encoderTime = publish_encoder_msg(msg)
        
        elif (self.state = CalibratingState()):
            modrive.calibrate()  # does the whole while this wait sequence
            front_state, back_state = modrive.get_current_state()

            # if both are idle it means its done calibrating
            if front_state == AXIS_STATE_IDLE \
                    and back_state == AXIS_STATE_IDLE:
                modrive.set_current_lim(100)
                modrive.set_velocity_ctrl()

        lock.release()


def publish_state_msg(msg, state):
    msg.state = states.index(state)
    msg.controller = int(sys.argv[1])
    lcm_.publish("/drivestatedata", msg.encode())
    print("changed state to " + state)


def publish_encoder_helper(msg, axis):
    msg.measuredCurrent = modrive.get_iq_measured(axis)
    msg.estimatedVel = modrive.get_vel_estimate(axis)

    motor_map = {("BACK", 0): 2, ("BACK", 1): 3,
                 ("FRONT", 0): 0, ("FRONT", 1): 1}
    msg.axis = motor_map[(axis, legalController)]

    lcm_.publish("/driveveldata", msg.encode())


def publish_encoder_msg(msg):
    if (legalAxis == "BOTH"):
        publish_encoder_helper(msg, "FRONT")
        publish_encoder_helper(msg, "BACK")
    else:
        publish_encoder_helper(msg, legalAxis)
    return t.time()


def drivestatecmd_callback(channel, msg):
    print("requested state call back is being called")
    global event
    global modrive

    controller_list = ["DISARMED", "ARMED", "CALIBRATING"]
    lock.acquire()
    message = DriveStateCmd.decode(msg)
    if message.controller == int(sys.argv[1]):  # Check which controller
        event = controller_list[message.state - 1]
    lock.release()


def drivevelcmd_callback(channel, msg):
    # if the program is in an ARMED state
    #   set the odrive's velocity to the float specified in the message
    # no state change
    axes = ["FRONT", "FRONT", "BACK", "BACK"]

    lock.acquire()
    global event
    global modrive
    global legalAxis

    message = DriveVelCmd.decode(msg)
    try:
        if(event == "ARMED"):
            modrive.closed_loop_ctrl()
            modrive.set_vel(axes[message.axis], message.vel)

    except AttributeError:
        print("odrive disconnected")

    lock.release()


if __name__ == "__main__":
    main()


class Modrive:
    CURRENT_LIM = 30

    odrive_axies = ["FRONT", "FRONT", "BACK", "BACK"]

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

    def set_current_lim(self, lim):
        self.front_axis.motor.config.current_lim = lim
        self.back_axis.motor.config.current_lim = lim

    # odrive.axis0.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

    def _set_control_mode(self, mode):
        self.front_axis.controller.config.control_mode = mode
        self.back_axis.controller.config.control_mode = mode

    def set_velocity_ctrl(self):
        self._set_control_mode(CTRL_MODE_VELOCITY_CONTROL)

    # odrive.axis0.motor.current_control.Iq_measured

    def get_iq_measured(self, axis):
        if (axis == "FRONT"):
            return self.front_axis.motor.current_control.Iq_measured
        elif(axis == "BACK"):
            return self.back_axis.motor.current_control.Iq_measured
        else:
            print("ERROR: cant get the measured iq for both motors at once")
            return 0

    # odrive.axis0.encoder.vel_estimate
    def get_vel_estimate(self, axis):
        # axis = self.odrive[axis_number]
        if (axis == "FRONT"):
            return self.front_axis.encoder.vel_estimate
        elif(axis == "BACK"):
            return self.back_axis.encoder.vel_estimate

    def calibrate(self):
        modrive._requested_state("FRONT", AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        while (modrive.get_current_state("FRONT") != AXIS_STATE_IDLE):
            t.sleep(0.1)
        modrive._requested_state("BACK", AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        while (modrive.get_current_state("BACK") != AXIS_STATE_IDLE):
            t.sleep(0.1)

    def idle(self):
        self._requested_state("BOTH", AXIS_STATE_IDLE)

    def closed_loop_ctrl(self):
        self._requested_state("BOTH", AXIS_STATE_CLOSED_LOOP_CONTROL)

    def _requested_state(self, axis, state):
        if (axis == "FRONT"):
            self.front_axis.requested_state = state
        elif (axis == "BACK"):
                self.back_axis.requested_state = state
        elif (axis == "BOTH"):
            self.back_axis.requested_state = state
            self.front_axis.requested_state = state

    # odrive.axis0.encoder.vel_estimate == 0

    def set_vel(self, axis, vel):
        if (axis == "FRONT"):
            self.front_axis.controller.vel_setpoint = vel
        elif axis == "BACK":
            self.back_axis.controller.vel_setpoint = vel
        else:
            print("ERROR, unknown axis")

    def get_current_state(self):
        return (self.front_axis.current_state, self.back_axis.current_state)

    def _reset(self, m_axis):
        m_axis.motor.config.pole_pairs = 15
        m_axis.motor.config.resistance_calib_max_voltage = 4
        m_axis.motor.config.requested_current_range = 25
        # Requires config save and reboot
        m_axis.motor.config.current_control_bandwidth = 100

        m_axis.encoder.config.mode = ENCODER_MODE_HALL
        m_axis.encoder.config.cpr = 90
        m_axis.encoder.config.bandwidth = 100
        m_axis.controller.config.pos_gain = 1
        m_axis.controller.config.vel_gain = 0.02
        m_axis.controller.config.vel_integrator_gain = 0.1
        m_axis.controller.config.vel_limit = 1000
        m_axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

    def reset(self):
        self._reset(self.front_axis)
        self._reset(self.back_axis)
        self.odrive.save_configuration()
        #  odv.dump_errors(odrive, True) clears errors from last reboot

    def check_errors(self):
        front = self.front_axis.error
        back = self.back_axis.error
        return back + front
