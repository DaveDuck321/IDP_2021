"""laptop_green controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import struct, math
import numpy as np
from queue import PriorityQueue

class RobotController:
    def __init__(self, emitter_channel, receiver_channel):
        self.robot = Robot()
        self.timestep = 1
        
        self.emitter = self.robot.getDevice("emitter")
        self.receiver = self.robot.getDevice("receiver")
        self.display = self.robot.getDevice("display")
        
        self.emitter.setChannel(emitter_channel)
        self.receiver.setChannel(receiver_channel)
        self.receiver.enable(self.timestep)
        self.display.setColor(2)
        
        self.dummy()
        self.process_data()
        
    def process_data(self):
        while self.robot.step(self.timestep) != -1:
            if self.receiver.getQueueLength() > 0:
                data = self.receiver.getData()
                front_dist, rear_dist, angle, bearing, pos_x, pos_z = struct.unpack("6f", data)
                self.receiver.nextPacket()
               
               
               
               
                
    def dummy(self):
        arena_map = np.zeros((240, 240), dtype = np.int32)
        arena_map[:20, :] = 1
        arena_map[-20:, :] = 1
        arena_map[:, :20] = 1
        arena_map[:, -20:] = 1
        start = (30, 30)
        goal = (60, 180)
        
        waypoints = self.calculate_route(arena_map, start, goal)
        data = struct.pack('f', len(waypoints))
        self.emitter.send(data)
        data = struct.pack('%sf' % len(waypoints), *waypoints)
        self.emitter.send(data)
        
        
        
        
    
    def calculate_route(self, arena_map, start, goal):
        walks = ((1, 0, 1), (1, 1, 2**0.5), (0, 1, 1), (-1, 1, 2**0.5), (-1, 0, 1), (-1, -1, 2**0.5), (0, -1, 1), (1, -1, 2**0.5))
        distances = -np.ones((arena_map.shape), dtype = np.float64)
        directions = -np.ones((arena_map.shape), dtype = np.int32)
        distances[start] = 0
        to_explore = PriorityQueue()
        to_explore.put((0, start))
        
        while not to_explore.empty():
            _, (cur_x, cur_z) = to_explore.get()
            if (cur_x, cur_z) == goal:
                print(distances[goal])
                break
            for direction, (x, z, dist) in enumerate(walks):
                if 0 <= cur_x + x < 240 and 0 <= cur_z + z < 240 and arena_map[cur_x + x, cur_z + z] != 1: # check if movement is in map and free to move into
                    if distances[cur_x + x, cur_z + z] == -1 or distances[cur_x, cur_z] + dist < distances[cur_x + x, cur_z + z]:
                        directions[cur_x + x, cur_z + z] = direction
                        manhattan_dist = ((cur_x + x - goal[0])**2 + (cur_z + z - goal[1])**2)**0.5
                        distances[cur_x + x, cur_z + z] = distances[cur_x, cur_z] + dist
                        to_explore.put((distances[cur_x + x, cur_z + z] + manhattan_dist, (cur_x + x, cur_z + z)))
        else:
            return []
            
        waypoints = [(goal[0] -120.0)/100.0, (goal[1] -120.0)/100.0]
        cur_x, cur_z = goal
        direction = directions[goal]
        while (cur_x, cur_z) != start:
            cur_x -= walks[direction][0]
            cur_z -= walks[direction][1]
            if directions[cur_x, cur_z] != direction:
                waypoints.append((cur_x -120.0)/100.0)
                waypoints.append((cur_z -120.0)/100.0)
                direction = directions[cur_x, cur_z]
        
        return waypoints

                

def main():
    green_laptop = RobotController(emitter_channel = 2, receiver_channel = 1)
    

if __name__ == "__main__":
    main()