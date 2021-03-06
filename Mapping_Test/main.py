import random
from functools import partial
import numpy as np
import pygame
from pygame import surfarray

BLOCK_WIDTH = 0.1  # 10 cm
ULTRASOUND_RANGE = 1.5  # 1.5 m
ULTRASOUND_NOISE = 0.2  # Standard deviation in distance measurement = 20cm / m
ULTRASOUND_ANGLE = (15 / 360) * 2 * np.pi  # 15 degrees FOV
WORLD_BOUNDS = np.array([2, 2])  # 2x2m stage
WORLD_RESOLUTION = 100  # px per meter (reduce for faster computation)
MAP_RESULTION = (
    2 * WORLD_RESOLUTION * WORLD_BOUNDS[0],
    2 * WORLD_RESOLUTION * WORLD_BOUNDS[1]
)

# For fast numpy computation
x = np.linspace(-WORLD_BOUNDS[0], WORLD_BOUNDS[0], MAP_RESULTION[0])
y = np.linspace(-WORLD_BOUNDS[1], WORLD_BOUNDS[1], MAP_RESULTION[1])
xx, yy = np.meshgrid(x, y)

# Generate all block positions for debugging
blocks = (np.random.rand(6, 2) - 0.5) * (1.6 * WORLD_BOUNDS)

# Datastructure containing the probabilities of finding a block in each location
probability_mask = 255*np.ones(MAP_RESULTION)
probability_map = np.zeros(MAP_RESULTION)
sensor_position = np.array([0, 0])

# Pygame stuff for visualization
pygame.init()
output_window = pygame.display.set_mode(MAP_RESULTION)


def ultrasound_pdf(mean, x):
    # Ultrasound gets less accurate with distance, this is gaussian
    sigma = mean * ULTRASOUND_NOISE
    return (1/(sigma * np.sqrt(2 * np.pi))) * np.exp(-0.5 * ((mean-x)/sigma) ** 2)


def to_screenspace(coords):
    return np.flip((coords + WORLD_BOUNDS)/(2 * WORLD_BOUNDS) * MAP_RESULTION)


def to_worldspace(_coords):
    coords = np.array([_coords[1], _coords[0]])
    return (2 * (coords / MAP_RESULTION) - 1) * WORLD_BOUNDS


def get_boarder_distance(sensor_pos, sensor_facing):
    # Top edge
    x_dir = np.cos(sensor_facing)
    y_dir = np.sin(sensor_facing)

    dist_right = (WORLD_BOUNDS[0] - sensor_pos[0])/x_dir
    dist_left = (-WORLD_BOUNDS[0] - sensor_pos[0])/x_dir

    dist_top = (WORLD_BOUNDS[1] - sensor_pos[1])/y_dir
    dist_bottom = (-WORLD_BOUNDS[1] - sensor_pos[1])/y_dir

    return min(filter(lambda x: x >= 0, [dist_right, dist_left, dist_top, dist_bottom]))


def ultrasound_reading(sensor_pos, sensor_facing):
    noise = np.random.normal(0, ULTRASOUND_NOISE, len(blocks))

    block_vectors = blocks - sensor_pos
    block_distances = np.linalg.norm(block_vectors, axis=1)
    block_distances += np.clip(block_distances*noise, 0, ULTRASOUND_RANGE)

    block_bearings = np.arctan2(block_vectors[:, 1], block_vectors[:, 0])
    block_bearing_range = BLOCK_WIDTH / (2 * block_distances)

    found_block = np.logical_and(
        block_distances < ULTRASOUND_RANGE,
        np.logical_or(
            abs(block_bearings - sensor_facing - block_bearing_range) < ULTRASOUND_ANGLE,
            abs(block_bearings - sensor_facing + block_bearing_range) < ULTRASOUND_ANGLE
        )
    )

    distance_from_wall = get_boarder_distance(sensor_pos, sensor_facing)

    distance_readings = [distance for (found, distance) in zip(found_block, block_distances) if found] + [
        ULTRASOUND_RANGE,
        np.random.normal(distance_from_wall, distance_from_wall * ULTRASOUND_NOISE)
    ]
    
    return min(distance_readings)


def map_scan_distances(scan_distances, scan_angles, scan_position):
    # Find guaranteed block angle: hlh detected
    pass  # to be implemented

    # Look at sensor readings: is a block in range
    for measurement_angle, detected_distance in zip(scan_angles, scan_distances):
        x_vectors = xx - scan_position[0]
        y_vectors = yy - scan_position[1]
        distances = np.hypot(x_vectors, y_vectors)
        bearings = np.abs(np.arctan2(y_vectors, x_vectors) - measurement_angle)

        # If reading is statistically indistinct (2 * sd) from wall, ignore it: it might be a wall
        distance_from_wall = get_boarder_distance(scan_position, measurement_angle)
        if abs(detected_distance - distance_from_wall) < 2 * distance_from_wall * ULTRASOUND_NOISE:
            continue

        if detected_distance < ULTRASOUND_RANGE:
            # Block detected (consider accurate) (ULTRASOUND_RANGE already includes safety factor)
            # Now mark this on a fuzzy probability map

            # Account for potential FOV distortion
            # Note: due to this, statistical values are purely relative
            ultrasound_fov_range = np.arange(-ULTRASOUND_ANGLE, ULTRASOUND_ANGLE, 1.0)

            fov_range = bearings < ULTRASOUND_ANGLE
            probabilities = ultrasound_pdf(detected_distance, distances)

            probability_map[:] += np.where(fov_range, probabilities, 0)
        else:
            # Mark exclusion reading on map with 90% certainty
            exclusion_mask = np.logical_and(distances < ULTRASOUND_RANGE - ULTRASOUND_NOISE * distances, bearings < 0.9 * ULTRASOUND_ANGLE)

            probability_mask[:] = np.where(exclusion_mask, 0.1 * probability_mask, probability_mask)


def do_scan(sensor_pos):
    # Mask decays to remove invalid readings
    probability_mask[:] = np.clip(probability_mask*1.2, 0.01, 255)

    scan_angles = np.arange(-np.pi, np.pi, 2 * ULTRASOUND_ANGLE)

    # Umm, technically still just python iteration but Felix wont mind
    make_readings = np.vectorize(partial(ultrasound_reading, sensor_pos))
    ultrasound_readings = make_readings(scan_angles)

    map_scan_distances(ultrasound_readings, scan_angles, sensor_pos)


def output_combined_probability_map():
    if pygame.key.get_pressed()[pygame.K_m]:
        # Debug: only display mask
        surfarray.blit_array(output_window, probability_mask)
        return
    
    if pygame.key.get_pressed()[pygame.K_d]:
        # Debug: only display probability density
        output_map = 255 * probability_map/np.amax(probability_map)
        surfarray.blit_array(output_window, output_map)
        return

    max_probability_intensity = np.amax(probability_map)
    output_map = probability_mask * (probability_map/max_probability_intensity)

    surfarray.blit_array(output_window, output_map)


do_scan(sensor_position)

should_exit = False
while not should_exit:
    output_combined_probability_map()

    # Draw sensor position
    pygame.draw.circle(output_window, (255, 255, 255), to_screenspace(sensor_position), 5, 1)

    if pygame.key.get_pressed()[pygame.K_s]:
        # Debug: draw true obstacle positions
        for block in blocks:
            pygame.draw.circle(output_window, (255, 0, 0), to_screenspace(block), 5, 1)
    
    if pygame.mouse.get_pressed()[0]:
        # Debug: move sensor and scan region
        mouse_pos = pygame.mouse.get_pos()
        coords = to_worldspace(mouse_pos)

        sensor_position = coords
        do_scan(sensor_position)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            should_exit = True

    pygame.display.flip()