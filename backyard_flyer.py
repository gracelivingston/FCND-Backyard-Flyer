import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5

class State:
    def __init__(self, drone):
        self._drone = drone
        print("Entering:", self.__class__.__name__)
        self.enter()

    @property
    def drone(self):
        return self._drone
    
    def enter(self):
        pass
    
    def local_position(self):
        pass

    def velocity(self):
        pass

    def transition(self):
        pass
    
class Manual(State):
    """The initial state which will transition to arminig."""

    def transition(self):
        self.drone.flight_state = Arming(self.drone)

class Arming(State):
    """Takes control of the drone and arms it, setting the home position."""

    def transition(self):
        """Prepare the unit for flight, then transition to Takeoff."""
        
        self.drone.take_control()
        self.drone.arm()
        self.drone.set_home_position(*self.drone.global_position)
        self.drone.flight_state = Takeoff(self.drone)
        
class Takeoff(State):
    def enter(self):
        """Set the target altitude and issue the takeoff command."""

        self.target_height = 3.0
        self.drone.takeoff(3.0)
        
    def local_position(self):
        """If we're at the right altitude, transition to Waypoint."""
        
        print("Takeoff: altitude", self.drone.local_position[2])
        if (-1.0 * self.drone.local_position[2] >
            0.95 * self.target_height):
            self.transition()

    def transition(self):
        """Start flying to waypoints."""

        self.drone.flight_state = Waypoint(self.drone)

class Waypoint(State):

    _target_position = None
    @property
    def box(self):
        return [
            [20.0, 0.0, 3.0],
            [20.0, 20.0, 3.0],
            [0.0, 20.0, 3.0],
            [0.0, 0.0, 3.0]
        ]

    def enter(self):
        self.waypoints = self.box
        self.target_position = self.box.pop(0)

    def get_target(self):
        return self._target_position

    def set_target(self, position):
        print("Waypoint: Navigating to waypoint:",
              position)
        self._target_position = position
        self.drone.cmd_position(
            self.target_position[0],
            self.target_position[1],
            self.target_position[2],
            0.0
            )

    target_position = property(get_target, set_target)

    def local_position(self):
        """See if we've run through our current target waypoint."""

        self.transition()


    def transition(self):
        """Select the next waypoint or start landing."""

        self.next_waypoint() or self.landing()

    def next_waypoint(self):
        """Select the next waypoint target if we've reached the last."""

        if len(self.waypoints) > 0:
            dX = np.linalg.norm(self.target_position[0:2] -
                            self.drone.local_position[0:2])
            print("Waypoint: dX:", dX)
            if dX < 1.0:
                print("Waypoint: Reached position:",
                    self.drone.local_position)
                
                self.target_position = self.waypoints.pop(0)

            return True

        print("Waypoint: all done")
        return False
    
    def landing(self):
        """Transition to Landing."""

        if np.linalg.norm(self.drone.local_velocity[0:2]) < 1.0:
            self.drone.flight_state = Landing(self.drone)

class Landing(State):
    """Land the drone by issuing the land command."""
    
    def __init__(self, drone):
        super(Landing, self).__init__(drone)
        self.drone.land()

    def velocity(self):
        if (self.drone.global_position[2] - self.drone.global_home[2] < 0.1
            and abs(self.drone.local_position[2]) < 0.01):
            self.transition()

    def transition(self):
        self.drone.flight_state = Disarming(self.drone)

class Disarming(State):
    def enter(self):
        self.drone.disarm()
        self.drone.release_control()

    def transition(self):
        if ~self.drone.armed & ~self.drone.guided:
            self.drone.stop()
            self.drone.in_mission = False
            self.drone.flight_state = Manual(self.drone)
   
            
class BackyardFlyer(Drone):
    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = Manual(self)

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """Call the local_position method on the current state."""

        self.flight_state.local_position()
        

    def velocity_callback(self):
        """Call the velocity method on the current state.

        This triggers when `MsgID.LOCAL_VELOCITY` is received and
        self.local_velocity contains new data

        """

        self.flight_state.velocity()

    def state_callback(self):
        """This triggers when `MsgID.STATE` is received and self.armed and
        self.guided contain new data

        This calls the transition method on the current flight state.
        """

        self.flight_state.transition()

    def arming_transition(self):
        """TODO: Fill out this method
        
        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")

    def takeoff_transition(self):
        """TODO: Fill out this method
        
        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")

    def waypoint_transition(self):
        """TODO: Fill out this method
    
        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        print("waypoint transition")

    def landing_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")

    def disarming_transition(self):
        """TODO: Fill out this method
        
        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")

    def manual_transition(self):
        """This method is provided
        
        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided
        
        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
