from pymavlink import mavutil
import time

# Start a connection listening on a TCP port
the_connection = mavutil.mavlink_connection('tcp:127.0.0.1:5764')

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Once connected, use 'the_connection' to get and send messages

time.sleep(1)

# put the drone in guided mode
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 4, 0, 0, 0, 0, 0)

# print the ACK
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
print(msg)

# ARM the drone
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

# print the ARM ACK if it's received
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
print(msg)

# take off to an altitude of 10m
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)

# print take off ack
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
print(msg)

# block until the drone has started moving
total_vel = 0
while total_vel < 0.05:
    msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    total_vel = abs(msg.vx) + abs(msg.vy) + abs(msg.vz)
    print("total velocity is %5.3f m/s" % total_vel)
# then block until it stops moving again
while total_vel > 0.05:
    msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    total_vel = abs(msg.vx) + abs(msg.vy) + abs(msg.vz)
    print("total velocity is %5.3f m/s" % total_vel)



# make the drone go 100 meters in front of its current position
the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b110111111000), 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

print("moving forward 100m")

# block until the drone has started moving
total_vel = 0
while total_vel < 0.05:
    msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    total_vel = abs(msg.vx) + abs(msg.vy) + abs(msg.vz)
    print("total velocity is %5.3f m/s" % total_vel)
# then block until it stops moving again
while total_vel > 0.05:
    msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    total_vel = abs(msg.vx) + abs(msg.vy) + abs(msg.vz)
    print("total velocity is %5.3f m/s" % total_vel)
    


# make the drone go 50 meters to the left of its position
the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, the_connection.target_system,
                        the_connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, int(0b110111111000), 0, -50, 0, 0, 0, 0, 0, 0, 0, 0, 0))

print("moving left 50m")

# block until the drone has started moving
total_vel = 0
while total_vel < 0.05:
    msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    total_vel = abs(msg.vx) + abs(msg.vy) + abs(msg.vz)
    print("total velocity is %5.3f m/s" % total_vel)
# then block until it stops moving again
while total_vel > 0.05:
    msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    total_vel = abs(msg.vx) + abs(msg.vy) + abs(msg.vz)
    print("total velocity is %5.3f m/s" % total_vel)



# return to the launch site and land
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 
                                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 10)
# print ack
msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
print(msg)
print("returning to launch site")

# block until the drone has started moving
total_vel = 0
while total_vel < 0.05:
    msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    total_vel = abs(msg.vx) + abs(msg.vy) + abs(msg.vz)
    print("total velocity is %5.3f m/s" % total_vel)
# then block until it stops moving again
while total_vel > 0.05:
    msg = the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
    total_vel = abs(msg.vx) + abs(msg.vy) + abs(msg.vz)
    print("total velocity is %5.3f m/s" % total_vel)