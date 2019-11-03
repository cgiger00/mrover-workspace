import lcm	
import sys
import time as t
import odrive as odv
from rover_msgs import ODriver_Req_State, ODriver_Req_Vel, ODriver_Pub_State, ODriver_Pub_Encoders
from odrive.enums import *

import Modrive

def main():
	lcm_ = lcm.LCM()

	#insert lcm subscribe statements here
	
	#These have been mapped
  
  
	lcm_.subscribe("/odriver_req_state", odriver_req_state_callback)
	
	lcm_.subscribe("/odrive_req_vel", odriver_req_vel_callback)

	global legalAxis
	legalAxis = sys.argv[2]
	
	while True:
		lcm_.handle()
		nextState()

	exit()
  
global states
states = ["BOOT", "DISARMED", "ARMED", "ERROR", "CALIBRATING", "EXIT"]
#Program states possible - BOOT, DISARMED, ARMED, ERROR, CALIBRATING, EXIT
#							1		 2	      3	    4		 5         6
currentState = "BOOT" #starting state
requestedState = "DISARMED" #starting requested state
odrive = None #starting odrive



def publish_state_msg(msg, state_number):
  currentState = states[state_number - 1]
  msg.state = state_number
  msg.serialid = sys.argv[1]
  lcm_.publish("/odriver_pub_state", msg1.encode()) #is lcm_ global? 
  return t.time()



def publish_encoder_helper(msg, axis):
  		msg.measuredCurrent = modrive.get_iq_measured(axis)
  	msg.estVel = modrive.get_vel_estimate(axis)
  	msg.serialid = sys.argv[1]
    if (axis == "RIGHT"):
      msg.axis = 'r'
    elif (axis == "LEFT"):
      msg.axis = 'l'
  	lcm_.publish("/odriver_pub_encoders", msg)
  
def publish_encoder_msg(msg)
	if (legalAxis == "BOTH"):
    publish_encoder_helper(msg, "LEFT")
    publish_encoder_helper(msg, "RIGHT")
    return t.time()
	elif (legalAxis == "RIGHT"):
    publish_encoder_helper(msg, "RIGHT")
    return t.time()
  elif (legalAxis == "LEFT"):
    publish_encoder_helper(msg, "RIGHT")
    return t.time()
  

def nextState():
	#every time the state changes, publish an odrive_state lcm message, with the new state
	global currentState
	global requestedState
	global odrive
	global encoderTime

  
	msg = odriver_pub_encoders()
	msg1 = odriver_pub_state()

	if (currentState == "BOOT"):
		#attempt to connect to odrive
		odrive = odv.find_any(serial_number=sys.argv[1])
		modrive = Modrive(odrive) #arguments = odr
    
		#Block until odrive is connected
		#set current limit on odrive to 50
		modrive.set_current_lim(legalAxis, 50)
		#set controller's control mode to velocity control
		modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
		#set currentState to DISARMED
		publish_state_msg(mgs1, 2)

	elif (currentState == "DISARMED"):
		#if 100 ms have passed since last time data was published
		#	publish an odrive_data lcm message with measured current and estimated velocity
		
		#Unsure if using correct timestamp
		if (encoderTime - t.time() > 0.1):
			encoderTime = publish_encoder_msg(msg)
      #check odrive's dump errors functionality

		
		
		#if there are errors
		#	set currentState to ERROR
		#else if requestedState is ARMED
		#	configure odrive for velocity control (starting with velocity 0)
		#	set currentState to ARMED
		#else if requested state is BOOT
		#	reboot the odrive
		#   set currentState to BOOT
		#else if requested state is CALIBRATING
		#	have the odrive recalibrate
		#	set currentState to CALIBRATING
    
    #unsure if this is right
		errors = odv.dump_errors(odrive)
		if errors:
      #sets state to error
			publish_state_msg(msg1, 4)
		elif requestedState == "ARMED":
      modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
      #sets state to armed
      publish_state_msg(mgs1, 3)
		elif requestedState == "BOOT":
			odrive.reboot()
      #sets state to boot 
      publish_state_msg(mgs1, 1)
		elif requestedState == "CALIBRATING":
      modrive.requested_state(legalAxis, AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
      #sets state to calibrating
      publish_state_msg(mgs1, 5)



	elif (currentState == "ARMED"):
		#check odrive's dump errors functionality
		#if there are errors
		#	set currentState to ERROR
		#else if requestedState is DISARMED
		#	configure odrive for idling
		#	set currentState to DISARMED
		#else if requestedState is BOOT
		#	reboot the odrive
		#	set currentState to BOOT
		#else if requested state is CALIBRATING
		#	have the odrive recalibrate
		#	set currentState to CALIBRATING

		if (encoderTime - t.time() > 0.1):
			encoderTime = publish_encoder_msg(msg)
      
		errors = odv.dump_errors(odrive)
		if errors:
      #sets state to error
      publish_state_msg(mgs1, 4)
		elif requestedState == "DISARMED":
			modrive.set_control_mode(legalAxis, AXIS_STATE_IDLE)
      #sets state to disarmed
      publish_state_msg(mgs1, 2)
		elif requestedState == "BOOT":
			odrive.reboot()
      #sets state to boot
      publish_state_msg(mgs1, 1)
		elif requestedState == "CALIBRATING":
			modrive.requested_state(legalAxis, AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
      #sets state to calibrating
      publish_state_msg(mgs1, 5)
		elif (currentState == "ERROR"):
		#if requestedState is BOOT
		#	reboot the odrive
		#	set currentState to BOOT
		#else if requested state is CALIBRATING
		#	have the odrive recalibrate
		#	set currentState to CALIBRATING
      if requestedState == "BOOT":
        odrive.reboot()
        #sets current state to boot
        publish_state_msg(mgs1, 1)
      elif requestedState == "CALIBRATING":
        modrive.requested_state(legalAxis, AXIS_STATE_FULL_CALIBRATION_SEQUENCE)
        #sets current state to calibrating
        publish_state_msg(mgs1, 5)
	elif (currentState == "CALIBRATING"):
		#if odrive is done calibrating
		#	set current limit on odrive to 50
		#	set controller's control mode to velocity control
		#	set currentState to DISARMED

		#We don't know how to check if done calibrating
		#if odrive.

		modrive.set_current_lim(legalAxis, 50)
		modrive.set_control_mode(legalAxis, CTRL_MODE_VELOCITY_CONTROL)
    #ssets state to disarmed 
    publish_state_msg(mgs1, 2)



def odriver_req_state_callback(channel,msg):
	message = ODriver_Req_State.decode(msg)
	if message.serialid == sys.argv[1]:
    requestedState = states[message.requestState - 1]
    if requestedState == "EXIT":
			if modrive.get_vel_estimate("LEFT") == 0 and modrive.get_current_state("LEFT") == AXIS_STATE_IDLE and \
      										modrive.get_vel_estimate("RIGHT") == 0 and modrive.get_current_state("RIGHT") == AXIS_STATE_IDLE:
					sys.exit()
			else:
          modrive.set_vel(legalAxis, 0)		
          modrive.requested_state(legalAxis, AXIS_STATE_IDLE)
					sys.exit()

    
	
def odriver_req_vel_callback(channel, msg):
	#if the program is in an ARMED state
	#	set the odrive's velocity to the float specified in the message
	#no state change
	message = ODriver_Req_Vel.decode(msg)
	if message.serialid == sys.argv[1] :
		if(currentState == "ARMED"):
      modrive.requested_state(legalAxis, AXIS_STATE_CLOSED_LOOP_CONTROL)
      modrive.set_vel(legalAxis, message.vel)			

if __name__ == "__main__":
	main()