import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

# Keep track of starting point
# When we detect an obstacle make a note of our yaw and return to that yaw after pickup
# When we detect we are stuck < 0.5 m/s try full throtle for 3 seconds or till we are above 1.0 m/s
# If we continue to be stuck try rotating our yaw to the right by 30 degrees for 3 seconds
# Once all 6 rocks are detected keep our heading to the starting point within 90 degrees of actual heading

    Rover.left_samp_angles = np.where(Rover.samp_angles * 180/np.pi > -10)[0]

    # Once we are done we need to get back to the starting position
    if Rover.samples_collected == 6:
        print("GO TO START")
        dist_start = np.sqrt((Rover.pos[0] - Rover.start_pos[0])**2 + (Rover.pos[1] - Rover.start_pos[1])**2)
        # Make sure we are heading in right general direction
        # TODO
        # If we are in 10 meters steer to starting point
   
        # If we are in 3 meters just stop
        if dist_start < 10.0 :
            print("10m From start")
            Rover.mode = 'stop'
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            return Rover
    print(Rover.samples_collected)

    # If we are picking_up a rock make a note of it 1 time
    if Rover.picking_up == 1:
        print("PU")
        if Rover.collecting == False:
            Rover.collecting = True
            Rover.samples_collected += 1
        return Rover
    else:
         Rover.collecting = False

    # If we are in grabbing distance stop as fast as possible
    if Rover.near_sample == 1:
        print("Stopping at sample")
        Rover.mode = 'stop'
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        if Rover.vel == 0:
            Rover.pick_up = True
        return Rover

    # If we are driving in a circle try to brake out by turning other way
    if Rover.mode == 'looping':
        print("Stopping looping")
        Rover.throttle = 0
        Rover.steer = -15
        Rover.brake = 0
        Rover.left_count += 1
        if Rover.left_count > 50: 
            Rover.mode = 'forward'
            Rover.left_count = 0
        return Rover

    if Rover.steer > 5:
        Rover.left_count += 1
    else:
        Rover.left_count = 0

    if Rover.left_count > 250:
        Rover.mode = 'looping'
        Rover.left_count = 0
        return Rover

    print(Rover.left_count)

    # If we are stuck try full throttle, they try rotating a bit to the right
    if Rover.mode == 'stuck':
        if Rover.stuck_mode == 'forward':
            print("Stuck Forward")
            Rover.throttle = 1
            #Rover.steer = 0
            Rover.stuck_counter += 1
            if Rover.stuck_counter > 45:
                Rover.stuck_mode = 'yaw'
                Rover.stuck_counter = 0
        elif Rover.stuck_mode == 'yaw':
            print("Stuck Yaw")
            Rover.throttle = 0
            Rover.steer = -15
            Rover.stuck_counter += 1
            if Rover.stuck_counter > 20:
                Rover.stuck_mode = 'forward'
                Rover.stuck_counter = 0

        if Rover.vel > 0.6:
            Rover.mode = 'forward'

        return Rover


    if Rover.mode == 'forward':
        if Rover.vel < 0.5:
            Rover.stuck_counter += 1
    else:
        Rover.stuck_counter = 0

    if Rover.stuck_counter > 90:
        Rover.mode = 'stuck'
        Rover.stuck_mode = 'forward'
        Rover.stuck_counter = 0

    # Get the left half of nav angles to wall crawl
    Rover.nav_angles = np.sort(Rover.nav_angles)[-int(len(Rover.nav_angles)/2):]

    # default navigation code
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':             

            # Check for samples to collect
            if len(Rover.left_samp_angles) > 1:
                print("Approaching sample")

                if Rover.vel < 0.75: #when finding sample approach slower
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0

                Rover.steer = np.clip(np.mean(Rover.samp_angles * 180/np.pi), -15, 15)            

            # Check the extent of navigable terrain
            elif len(Rover.nav_angles) >= Rover.stop_forward:  
                print("Moving forward")
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    print("No where to go stopped")
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                print("Stopping")
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:

                            # Check for samples to collect
                if len(Rover.left_samp_angles) > 1:
                    print("Approaching sample")
                    Rover.steer = np.clip(np.mean(Rover.samp_angles * 180/np.pi), -15, 15)            
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0                    

                    # Now we're stopped and we have vision data to see if there's a path forward
                elif len(Rover.nav_angles) < Rover.go_forward:
                    print("Can go forward")
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    print("Go forward")
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        print("Drive")
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    return Rover

