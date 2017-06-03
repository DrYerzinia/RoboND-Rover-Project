import numpy as np
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, above_rgb_thresh=(160, 160, 160), below_rgb_thresh=(255, 255, 225)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    matched = (img[:,:,0] > above_rgb_thresh[0])      \
                & (img[:,:,0] <= below_rgb_thresh[0]) \
                & (img[:,:,1] > above_rgb_thresh[1])  \
                & (img[:,:,1] <= below_rgb_thresh[1]) \
                & (img[:,:,2] > above_rgb_thresh[2])  \
                & (img[:,:,2] <= below_rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[matched] = 1
    # Return the binary image
    return color_select

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # TODO:
    # Convert yaw to radians
    # Apply a rotation
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = xpix*np.cos(yaw_rad)-ypix*np.sin(yaw_rad)
    ypix_rotated = xpix*np.sin(yaw_rad)+ypix*np.cos(yaw_rad)
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):

    if Rover.start_pos == None:
        Rover.start_pos = Rover.pos

    if Rover.perception_count == None:
        Rover.perception_count = 0

    image = Rover.img
    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    bottom_offset = 6
    src = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    dst = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    
    map_view = perspect_transform(Rover.img, src, dst)

    # middle between brightest sky and darkest ground from sample image
    navigable_thresholds = (160, 160, 160);
    #navigable_thresholds = (190, 178, 167);

    navigable = color_thresh(map_view, navigable_thresholds)

    obs = color_thresh(map_view, (0,0,0), navigable_thresholds)
    
    # Within 20 of high/low values from sample image
    rock_low_thresholds = (127, 94, 0) # 147 114 9   212 180 57
    rock_high_thresholds = (232, 200, 77)

    # Not flat... maybe should do things a bit different
    sample_detect = color_thresh(Rover.img, rock_low_thresholds, rock_high_thresholds)
    sample_detect = perspect_transform(sample_detect, src, dst)

    Rover.vision_image[:,:,0] = obs*255
    Rover.vision_image[:,:,1] = sample_detect*255
    Rover.vision_image[:,:,2] = navigable*255

    nav_xpix, nav_ypix = rover_coords(navigable)
    obs_xpix, obs_ypix = rover_coords(obs)
    samp_xpix, samp_ypix = rover_coords(sample_detect)

    nav_x, nav_y = pix_to_world(nav_xpix, nav_ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 10)
    obs_x, obs_y = pix_to_world(obs_xpix, obs_ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 10)
    samp_x, samp_y = pix_to_world(samp_xpix, samp_ypix, Rover.pos[0], Rover.pos[1], Rover.yaw, Rover.worldmap.shape[0], 10)

    # Update world map if we are not tilted more than 0.5 deg
    if (Rover.roll < 0.5 or Rover.roll > 359.5) or (Rover.pitch < 0.5 or Rover.pitch > 359.5):

        Rover.worldmap[obs_y, obs_x, 0] += 1;
        Rover.worldmap[samp_y, samp_x, 1] += 1;
        Rover.worldmap[nav_y, nav_x, 2] += 1;

    # Clear out low quality nav pixles
    # Delete pixels less than the mean over three
    if(Rover.perception_count % 200 == 0):
        nav_pix = Rover.worldmap[:,:,2] > 0
        lowqual_pix = Rover.worldmap[:,:,2] < np.mean(Rover.worldmap[nav_pix, 2]) / 4
        Rover.worldmap[lowqual_pix, 2] = 0

    dists, angles = to_polar_coords(nav_xpix, nav_ypix)

    Rover.nav_dists = dists
    Rover.nav_angles = angles

    samp_dists, samp_angles = to_polar_coords(samp_xpix, samp_ypix)

    Rover.samp_dists = samp_dists
    Rover.samp_angles = samp_angles
    
    Rover.perception_count += 1

    return Rover
