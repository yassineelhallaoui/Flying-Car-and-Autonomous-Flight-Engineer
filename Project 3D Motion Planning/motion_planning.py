import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import random

from planning_utils import *
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

import matplotlib.pyplot as plt

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 8

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        import csv
        with open('colliders.csv', newline='') as f:
            reader = csv.reader(f)
            row1 = next(reader)  # gets the first line

        lat0 = float(row1[0].split()[1])
        lon0 = float(row1[1].split()[1])
       
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        drone_global_position = self.global_position
 
        # TODO: convert to current local position using global_to_local()
        drone_local_position = global_to_local(drone_global_position, self.global_home)
        
        print('global home {0}, global position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))

        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        #grid_start = (-north_offset, -east_offset)
        
        # TODO: convert start position to current position rather than map center
        grid_start = (int(self.local_position[0] - north_offset) , int(self.local_position[1] - east_offset))
        
        # Set goal as some arbitrary position on the grid
        #grid_goal = (-north_offset + 10, -east_offset + 10)
        
        # TODO: adapt to set goal as latitude / longitude position and convert
        # goal global coordinates --> Lon: 37.793658, Lat: -122.398028
        #grid_goal = global_to_local(goal_global, self.global_home)
        north_random_distance = random.randint(-abs(north_offset), abs(north_offset)) 
        east_random_distance = random.randint(-abs(east_offset), abs(east_offset))
        grid_goal = (grid_start[0] + north_random_distance, grid_start[1] + east_random_distance)

        while grid[grid_goal[0], grid_goal[1]] == 1:
            north_random_distance = random.randint(-abs(north_offset), abs(north_offset)) 
            east_random_distance = random.randint(-abs(east_offset), abs(east_offset))
            grid_goal = (grid_start[0] + north_random_distance, grid_start[1] + east_random_distance)

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        # TODO: prune path to minimize number of waypoints
        prune_path = self.prune_path(path)
        # NOTE (if you're feeling ambitious): Try a different approach altogether!
        


        # visualize the path generated.Close plot window to proceed execution
        plt.imshow(grid, cmap='Greys', origin='lower')
        plt.plot(grid_start[1], grid_start[0], 'x')
        plt.plot(grid_goal[1], grid_goal[0], 'x')
        pp = np.array(prune_path)
        plt.plot(pp[:, 1], pp[:, 0], 'g')
        plt.scatter(pp[:, 1], pp[:, 0])
        plt.grid(True)
        plt.xlabel('EAST')
        plt.ylabel('NORTH')
        plt.show()
        
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in prune_path]
        # Set self.waypoints
        self.waypoints = waypoints
        # Send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()
        
    def prune_path(self,path):
        epsilon = 5
        def point(p):
            return np.array([p[0], p[1], 1.]).reshape(1, -1)
        def collinearity_check(p1, p2, p3, epsilon):
            m = np.concatenate((p1, p2, p3), 0)
            det = np.linalg.det(m)
            return abs(det) < epsilon   
        
        waypoints = [p for p in path] #makes a copy of the 'path' list  
    
        i = 0
        while i < len(waypoints) - 2:
            p1 = point(waypoints[i])
            p2 = point(waypoints[i+1])
            p3 = point(waypoints[i+2])
        
            # If the 3 points are in a line remove
            # the 2nd point.
            # The 3rd point now becomes and 2nd point
            # and the check is redone with a new third point
            # on the next iteration.
            if collinearity_check(p1, p2, p3, epsilon):
                # Something subtle here but we can mutate
                # `pruned_path` freely because the length
                # of the list is check on every iteration.
                waypoints.remove(waypoints[i+1])
            else:
                i += 1
        return waypoints

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")

    parser.add_argument('--lat', type=float, default=1000, help="latitude")
    parser.add_argument('--lon', type=float, default=1000, help="latitude")

    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
