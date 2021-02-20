import random
from functools import partial
import numpy as np
import pygame
from pygame import surfarray

BLOCK_WIDTH = 0.1  # 10 cm
ULTRASOUND_RANGE = 1.5  # 1 m
ULTRASOUND_NOISE = 0.05  # Standard deviation = 5cm / m
ULTRASOUND_ANGLE = (10 / 360) * 2 * np.pi  # 5 degrees
WORLD_BOUNDS = np.array([2, 2])  # 2x2 m stage
WORLD_RESOLUTION = 100  # px per meter
MAP_RESULTION = (
    2 * WORLD_RESOLUTION * WORLD_BOUNDS[0],
    2 * WORLD_RESOLUTION * WORLD_BOUNDS[1]
)

# For computation
x = np.linspace(-WORLD_BOUNDS[0], WORLD_BOUNDS[0], MAP_RESULTION[0])
y = np.linspace(-WORLD_BOUNDS[1], WORLD_BOUNDS[1], MAP_RESULTION[1])
xx, yy = np.meshgrid(x, y)

# A pain but felix wants the vectors
# (despite worse time complexity in this case)
# (Probably still 50x faster tho)
blocks = (np.random.rand(6, 2) - 0.5) * (2 * WORLD_BOUNDS)

map_data = 255*np.ones(MAP_RESULTION)

pygame.init()
window = pygame.display.set_mode(MAP_RESULTION)


def to_screenspace(coords):
    return np.flip((coords + WORLD_BOUNDS)/(2 * WORLD_BOUNDS) * MAP_RESULTION)

def to_worldspace(_coords):
    coords = np.array([_coords[1], _coords[0]])
    return (2 * (coords / MAP_RESULTION) - 1) * WORLD_BOUNDS

def ultrasound_reading(color, sensor_pos, sensor_facing):
    noise = np.random.normal(0, ULTRASOUND_NOISE, len(blocks))

    block_vectors = blocks - sensor_pos
    block_distances = np.linalg.norm(block_vectors, axis=1)
    block_bearings = np.arctan2(block_vectors[:, 1], block_vectors[:, 0])
    block_bearing_range = BLOCK_WIDTH / (2 * block_distances)

    found_block = np.logical_and(
        block_distances < ULTRASOUND_RANGE,
        np.logical_or(
            abs(block_bearings - sensor_facing - block_bearing_range) < ULTRASOUND_ANGLE,
            abs(block_bearings - sensor_facing + block_bearing_range) < ULTRASOUND_ANGLE
        )
    ).any()

    #distance_readings = [distance for (found, distance) in zip(found_block, block_distances) if found] + [min(ULTRASOUND_RANGE, )]

    #min_reading = min(distance_readings)
    #print(distance_readings)

    if found_block:
        return

    x_vectors = xx - sensor_pos[0]
    y_vectors = yy - sensor_pos[1]
    distances = np.hypot(x_vectors, y_vectors)
    bearings = np.abs(np.arctan2(y_vectors, x_vectors) - sensor_facing)

    hidden_mask = np.logical_and(distances < ULTRASOUND_RANGE, bearings < ULTRASOUND_ANGLE)

    map_data[:] = np.where(hidden_mask, color, map_data)


def do_scan(sensor_pos):
    scan_angles = np.arange(-np.pi, np.pi, 2 * ULTRASOUND_ANGLE)

    # Umm, technically still just python iteration but felix wont mind
    make_readings = np.vectorize(partial(ultrasound_reading, 0, sensor_pos))
    make_readings(scan_angles)


sensor_position = np.array([0, 0])

should_exit = False
do_scan(sensor_position)

while not should_exit:
    surfarray.blit_array(window, map_data)

    pygame.draw.circle(window, (255, 255, 255), to_screenspace(sensor_position), 5, 1)
    for block in blocks:
        pygame.draw.circle(window, (255, 0, 0), to_screenspace(block), 5, 1)
    
    if pygame.mouse.get_pressed()[0]:
        mouse_pos = pygame.mouse.get_pos()
        coords = to_worldspace(mouse_pos)

        sensor_position = coords
        do_scan(sensor_position)
        print(coords, " at angle ", np.arctan2(coords[1], coords[0]))

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            should_exit = True

    pygame.display.update()