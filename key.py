from pymavlink import mavutil
import time
import argparse  
import keyboard
import evdev

from utilities.connect_to_sysid import connect_to_sysid
from utilities.wait_for_position_aiding import wait_until_position_aiding
from utilities.get_autopilot_info import get_autopilot_info

main_mode_mapping_px4 = {
    'MANUAL': 0,
    'ALTCTL': 1,
    'POSCTL': 2,
    'AUTO': 3,
    'ACRO': 4,
    'OFFBOARD': 5,
    'STABILIZED': 6,
    'RATTITUDE': 7,
}

sub_mode_mapping_px4 = {
    'READY': 0,
    'TAKEOFF': 1,
    'HOLD': 2,  # LOITER in MAVLink
    'MISSION': 3,
    'RETURN_TO_LAUNCH': 4,
    'LAND': 5,
    'FOLLOW_ME': 6,
}

def takeoff(mav_connection, takeoff_altitude: float, tgt_sys_id: int = 1, tgt_comp_id=1):

    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    wait_until_position_aiding(mav_connection)

    autopilot_info = get_autopilot_info(mav_connection, tgt_sys_id)

    if autopilot_info["autopilot"] == "ardupilotmega":
        print("Connected to ArduPilot autopilot")
        mode_id = mav_connection.mode_mapping()["GUIDED"]
        takeoff_params = [0, 0, 0, 0, 0, 0, takeoff_altitude]

    elif autopilot_info["autopilot"] == "px4":
        print("Connected to PX4 autopilot")
        print(mav_connection.mode_mapping())
        mode_id = mav_connection.mode_mapping()["TAKEOFF"][1]
        print(mode_id)
        msg = mav_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        starting_alt = msg.alt / 1000
        takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + takeoff_altitude]

    else:
        raise ValueError("Autopilot not supported")


    # Change mode to guided (Ardupilot) or takeoff (PX4)
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK:  {ack_msg}")

    # Arm the UAS
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    arm_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Arm ACK:  {arm_msg}")

    # Command Takeoff
    mav_connection.mav.command_long_send(tgt_sys_id, tgt_comp_id,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3], takeoff_params[4], takeoff_params[5], takeoff_params[6])

    takeoff_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Takeoff ACK:  {takeoff_msg}")

    return takeoff_msg.result




def arm(mav_connection):
    # Wait for the first heartbeat
    # This sets the system and component ID of remote system for the link
    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    # return the result of the ACK message
    return msg.result
    
def disarm(mav_connection):
    # Wait for the first heartbeat
    # This sets the system and component ID of remote system for the link
    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)

    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    # return the result of the ACK message
    return msg.result
    
def land(mav_connection, timeout: int = 10) -> int:
    """
    Sends a command for the drone to land.

    Args:
        the_connection (mavutil.mavlink_connection): The MAVLink connection to use.
        timeout (int): Time in seconds to wait for an acknowledgment.

    Returns:
        int: mavutil.mavlink.MAV_RESULT enum value.
    """

    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))	
    # Send a command to land
    mav_connection.mav.command_long_send(
        mav_connection.target_system, 
        mav_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 
        0, 0, 0, 0, 0, 0, 0, 0
    )

    # Wait for the acknowledgment
    ack = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=timeout)
    if ack is None:
        print('No acknowledgment received within the timeout period.')
        return None

    return ack.result


def change_mode(mav_connection, mode, autopilot='ardupilot', sub_mode='NONE'):
	
    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))	
    
    if autopilot == 'ardupilot':
        # Check if mode is available
        if mode not in mav_connection.mode_mapping():
            print(f'Unknown mode : {mode}')
            print(f"available modes: {list(master.mode_mapping().keys())}")
            raise Exception('Unknown mode')
        
        # Get mode ID
        mode_id = mav_connection.mode_mapping()[mode]
        sub_mode = 0
    elif autopilot == 'px4':
        # Get mode ID
        mode_id = main_mode_mapping_px4[mode]
        sub_mode = sub_mode_mapping_px4[sub_mode]


    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, sub_mode, 0, 0, 0, 0)
                                
    ack_msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    
    if ack_msg is None:
        print('No acknowledgment received within the timeout period.')
        return None
    
    print(ack_msg)
    return ack_msg.result

def servo(mav_connection):
    # Wait for the first heartbeat
    # This sets the system and component ID of remote system for the link
    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 9, 1900, 0, 0, 0, 0, 0)

    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    # return the result of the ACK message
    return msg.result
    
def camera(mav_connection):
    # Wait for the first heartbeat
    # This sets the system and component ID of remote system for the link
    mav_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
          (mav_connection.target_system, mav_connection.target_component))

    mav_connection.mav.command_long_send(mav_connection.target_system, mav_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE, 0, 0, 60, 0, 0, 0, 0, 0)

    msg = mav_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    # return the result of the ACK message
    return msg.result

    
    
if __name__ == "__main__":
	parser = argparse.ArgumentParser(description='Send arm/disarm commands using MAVLink protocol.')
	parser.add_argument('-c', '--connect', help="Connection string", default='127.0.0.1:14550')
	parser.add_argument("--altitude", type=int, help="Altitude for the UAV to reach upon takeoff.", default=10)
	parser.add_argument("--sysid", type=int, help="System ID of the UAV to command.", default=1)
	parser.add_argument('--timeout', type=int, default=10, help='Timeout in seconds to wait for a command acknowledgment.')
	# parser.add_argument('-a', '--arm', type=int, choices=[0, 1], help="Arm/Disarm command", default=1)
    
	args = parser.parse_args()
	mav_connection = mavutil.mavlink_connection(args.connect)
	device = evdev.InputDevice("/dev/input/event11")

	for event in device.read_loop(): 
		
		if event.type==1 and event.code == 37:
			result = arm(mav_connection)
			print(f'Result of arm command: {result}')
			#time.sleep(2)
			#while keyboard.is_pressed('k'):  # Wait for 'k' to be released before continuing
			#	pass
		elif event.type==1 and event.code == 32:
			result = disarm(mav_connection)
			print(f'Result of disarm command: {result}')
			#time.sleep(2)
			#while keyboard.is_pressed('d'):  # Wait for 'd' to be released before continuing
			#	pass
		elif event.type==1 and event.code == 20:
			result = takeoff(mav_connection,args.altitude)
			print(f'Result of takeoff command: {result}')
			#time.sleep(2)
			#while keyboard.is_pressed('t'):  # Wait for 't' to be released before continuing
			#	pass
		elif event.type==1 and event.code == 38:
			result = land(mav_connection,args.timeout)
			print(f'Result of land command: {result}')
			#time.sleep(2)
			#while keyboard.is_pressed('l'):  # Wait for 'l' to be released before continuing
			#	pass
		elif event.type==1 and event.code == 44:
			result =  change_mode(mav_connection, "QSTABILIZE", "ardupilot", "READY")
			print(f'Result of mode change command: {result}')
			#time.sleep(2)
			#while keyboard.is_pressed('z'):  # Wait for 'l' to be released before continuing
			#	pass
		elif event.type==1 and event.code == 45:
			result =  change_mode(mav_connection, "QLOITER", "ardupilot", "READY")
			#print(f'Result of mode change command: {result}')
			#time.sleep(2)
			#while keyboard.is_pressed('x'):  # Wait for 'l' to be released before continuing
			#	pass
		elif event.type==1 and event.code == 46:
			result =  change_mode(mav_connection, "FBWA", "ardupilot", "READY")
			print(f'Result of mode change command: {result}')
			#time.sleep(2)
			#while keyboard.is_pressed('c'):  # Wait for 'l' to be released before continuing
			#	pass
		elif event.type==1 and event.code == 47:
			result =  change_mode(mav_connection, "FBWB", "ardupilot", "READY")
			print(f'Result of mode change command: {result}')
			#time.sleep(2)
			#while keyboard.is_pressed('v'):  # Wait for 'l' to be released before continuing
			#	pass
		elif event.type==1 and event.code == 48:
			result =  change_mode(mav_connection, "CRUISE", "ardupilot", "READY")
			print(f'Result of mode change command: {result}')
			#time.sleep(2)
			#while keyboard.is_pressed('b'):  # Wait for 'l' to be released before continuing
			#	pass'''
	
		elif event.type==1 and event.code == 34:
			result =  change_mode(mav_connection, "GUIDED", "ardupilot", "READY")
			print(f'Result of mode change command: {result}')
			# time.sleep(2)
			#while keyboard.is_pressed('g'):  # Wait for 'l' to be released before continuing
			#	pass
		elif event.type==1 and event.code == 35:
			result =  change_mode(mav_connection, "LOITER", "ardupilot", "READY")
			print(f'Result of mode change command: {result}')
			# time.sleep(2)
			#while keyboard.is_pressed('h'):  # Wait for 'l' to be released before continuing
			#	pass
		elif event.type==1 and event.code == 31:
			result =  change_mode(mav_connection, "STABILIZE", "ardupilot", "READY")
			print(f'Result of mode change command: {result}')
			# time.sleep(2)
			#while keyboard.is_pressed('s'):  # Wait for 'l' to be released before continuing
			# 	pass
		elif event.type==1 and event.code == 16:
			result =  servo(mav_connection)
			print(f'Result of servo trigger command: {result}')
			time.sleep(2)
			#while keyboard.is_pressed('p'):  # Wait for 'l' to be released before continuing
			#	pass
		elif event.type==1 and event.code == 25:
			result =  camera(mav_connection)
			print(f'Result of camera trigger command: {result}')
			time.sleep(2)
			#while keyboard.is_pressed('q'):  # Wait for 'l' to be released before continuing
			#	pass



