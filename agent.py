mport carla
import time
import numpy as np
# import matplotlib.pyplot as plt
import math
    
class Agent():
    def __init__(self, vehicle=None):
        self.vehicle = vehicle

        #control angle
        self.control_angle = 0

        # PID Variables
        self.integrator = 0
        self.prev_error = 0
        self.differentiator = 0
        self.prev_meas = 0

        self.current_lane = 1

        self.a_star = False

    def run_step(self, filtered_obstacles, waypoints, vel, transform, boundary):
        """
        Execute one step of navigation.

        Args:
        filtered_obstacles
            - Type:        List[carla.Actor(), ...]
            - Description: All actors except for EGO within sensoring distance
        waypoints 
            - Type:         List[[x,y,z], ...] 
            - Description:  List All future waypoints to reach in (x,y,z) format
        vel
            - Type:         carla.Vector3D 
            - Description:  Ego's current velocity in (x, y, z) in m/s
        transform
            - Type:         carla.Transform 
            - Description:  Ego's current transform
        boundary 
            - Type:         List[List[left_boundry], List[right_boundry]]
            - Description:  left/right boundary each consists of 20 waypoints,
                            they defines the track boundary of the next 20 meters.

        Return: carla.VehicleControl()
        """
        # Actions to take during each simulation step
        # Feel Free to use carla API; however, since we already provide info to you, using API will only add to your delay time
        # Currently the timeout is set to 10s

        # 
        print("Reach Customized Agent")

        #In the future the future way_point list will be the output of our planning algorithm
        #This one is using the middle of the lane -- much better for stability + finishes all tracks
        left_lane = []
        middle_lane = []
        right_lane = []
        left_lane_obs, right_lane_obs = [], []
        for i in range(len(boundary[0])):
            left_lane.append([boundary[0][i].transform.location.x + 25*(boundary[1][i].transform.location.x - boundary[0][i].transform.location.x)/100,
                           boundary[0][i].transform.location.y + 25*(boundary[1][i].transform.location.y - boundary[0][i].transform.location.y)/100])
            middle_lane.append([(boundary[0][i].transform.location.x + boundary[1][i].transform.location.x)/2,
                           (boundary[0][i].transform.location.y + boundary[1][i].transform.location.y)/2])
            right_lane.append([boundary[0][i].transform.location.x + 75*(boundary[1][i].transform.location.x - boundary[0][i].transform.location.x)/100,
                           boundary[0][i].transform.location.y + 75*(boundary[1][i].transform.location.y - boundary[0][i].transform.location.y)/100])

        for i in range(50):
            left_lane_obs.append([boundary[0][i].transform.location.x + (boundary[1][i].transform.location.x - boundary[0][i].transform.location.x)/4,
                           boundary[0][i].transform.location.y + (boundary[1][i].transform.location.y - boundary[0][i].transform.location.y)/4])
            right_lane_obs.append([boundary[0][i].transform.location.x + 3*(boundary[1][i].transform.location.x - boundary[0][i].transform.location.x)/4,
                           boundary[0][i].transform.location.y + 3*(boundary[1][i].transform.location.y - boundary[0][i].transform.location.y)/4])
        #This one is using waypoints -- not very good + doesn't finish all tracks 
        # future_waypoint = waypoints[0] # currently just waypoints, will be decided by planning algorithm later
        
        #Astar Attempt 1
        # start and goal position
        sx = transform.location.x
        sy = transform.location.y
        
        gx = (boundary[0][10].transform.location.x + boundary[1][10].transform.location.x)/2
        gy = (boundary[0][10].transform.location.y + boundary[1][10].transform.location.y)/2

        grid_size = 1  # [m]
        robot_radius = 2.5 # [m]

        # set obstacle positions
        ox, oy = [], []
        left_occupied, right_occupied, middle_occupied = 0, 0, 0
        #print(transform)
        for i in range(15): #len(boundary[0])
            ox.append(boundary[0][i].transform.location.x )#+ abs(5*np.cos(transform.rotation.yaw)))
            ox.append(boundary[1][i].transform.location.x )#- abs(5*np.cos(transform.rotation.yaw)))
            oy.append(boundary[0][i].transform.location.y )#+ abs(5*np.sin(transform.rotation.yaw)))
            oy.append(boundary[1][i].transform.location.y )#- abs(5*np.sin(transform.rotation.yaw)))

        self.astar = False
        stopped_vel = -1
        if len(filtered_obstacles) > 0:
            for i in range(len(filtered_obstacles)):
                loc = filtered_obstacles[i].get_location()

                for i in range(40):
                    if(np.sqrt((left_lane_obs[i][0] - loc.x)**2 + (left_lane_obs[i][1] - loc.y)**2) < 1.75):
                        left_occupied = 1
                        self.astar = True
                        if(self.current_lane == 0 and i <= 8):
                            stopped_vel = 0
                        #print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")

                    if(np.sqrt((right_lane_obs[i][0] - loc.x)**2 + (right_lane_obs[i][1] - loc.y)**2) < 1.75):
                        right_occupied = 1
                        self.astar = True
                        if(self.current_lane == 2 and i <= 8):
                            stopped_vel = 0
                        #print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
                    if(np.sqrt((middle_lane[i][0] - loc.x)**2 + (middle_lane[i][1] - loc.y)**2) < 1.75):
                        middle_occupied = 1
                        self.astar = True
                        if(self.current_lane == 1 and i <= 8):
                            stopped_vel = 0
                        #print("(((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((())))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))))")
                    
                ox.append(loc.x)
                oy.append(loc.y)

                    # ox.append(loc.x+1)
                    # oy.append(loc.y)

                    # ox.append(loc.x-1)
                    # oy.append(loc.y)

                    # ox.append(loc.x)
                    # oy.append(loc.y+1)

                    # ox.append(loc.x)
                    # oy.append(loc.y-1)

                    # ox.append(loc.x+1)
                    # oy.append(loc.y+1)

                    # ox.append(loc.x-1)
                    # oy.append(loc.y+1)

                    # ox.append(loc.x+1)
                    # oy.append(loc.y-1)

                    # ox.append(loc.x-1)
                    # oy.append(loc.y-1)

                    # ox.append(loc.x+2)
                    # oy.append(loc.y)

                    # ox.append(loc.x-2)
                    # oy.append(loc.y)

                    # ox.append(loc.x)
                    # oy.append(loc.y+2)

                    # ox.append(loc.x)
                    # oy.append(loc.y+2)

                    # ox.append(loc.x)
                    # oy.append(loc.y+3)

                    # ox.append(loc.x)
                    # oy.append(loc.y-3)

                    # ox.append(loc.x+3)
                    # oy.append(loc.y)

                    # ox.append(loc.x+3)
                    # oy.append(loc.y)

                    # 
                    
        if (left_occupied == 0 and middle_occupied == 0 and right_occupied == 0):
            if(self.current_lane == 0):
                gx = left_lane[10][0]
                gy = left_lane[10][1]
            elif(self.current_lane == 1):
                gx = middle_lane[10][0]
                gy = middle_lane[10][1]
            elif(self.current_lane == 2):
                gx = right_lane[10][0]
                gy = right_lane[10][1]
        elif (left_occupied == 0 and middle_occupied == 0 and right_occupied == 1):
            if(self.current_lane == 0):
                gx = left_lane[10][0]
                gy = left_lane[10][1]
                self.current_lane = 0
            elif(self.current_lane == 1):
                gx = middle_lane[10][0]
                gy = middle_lane[10][1]
                self.current_lane = 1
        elif (left_occupied == 0 and middle_occupied == 1 and right_occupied == 0):
            if(self.current_lane == 0):
                gx = left_lane[10][0]
                gy = left_lane[10][1]
                self.current_lane = 0
            elif(self.current_lane == 2):
                gx = right_lane[10][0]
                gy = right_lane[10][1]
                self.current_lane = 2
            elif self.current_lane == 1:
                if(self.control_angle <= 0):
                    gx = left_lane[10][0]
                    gy = left_lane[10][1]
                    self.current_lane = 0
                elif(self.control_angle > 0):
                    gx = right_lane[10][0]
                    gy = right_lane[10][1]
                    self.current_lane = 2
        elif (left_occupied == 0 and middle_occupied == 1 and right_occupied == 1):
            gx = left_lane[10][0]
            gy = left_lane[10][1]
            self.current_lane = 0
        elif (left_occupied == 1 and middle_occupied == 0 and right_occupied == 0):
            if(self.current_lane == 0):
                gx = middle_lane[10][0]
                gy = middle_lane[10][1]
                self.current_lane = 1
            elif(self.current_lane == 2):
                gx = right_lane[10][0]
                gy = right_lane[10][1]
                self.current_lane = 2
            elif self.current_lane == 1:
                gx = middle_lane[10][0]
                gy = middle_lane[10][1]
                self.current_lane = 1
        elif (left_occupied == 1 and middle_occupied == 0 and right_occupied == 1):
            gx = middle_lane[10][0]
            gy = middle_lane[10][1]
            self.current_lane = 1
        elif (left_occupied == 1 and middle_occupied == 1 and right_occupied == 0):
            gx = right_lane[10][0]
            gy = right_lane[10][1]
            self.current_lane = 2
        elif (left_occupied == 1 and middle_occupied == 1 and right_occupied == 1):
            # #print("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^")
            if(self.current_lane == 0):
                gx = left_lane[10][0]
                gy = left_lane[10][1]
            elif(self.current_lane == 1):
                gx = middle_lane[10][0]
                gy = middle_lane[10][1]
            elif(self.current_lane == 2):
                gx = right_lane[10][0]
                gy = right_lane[10][1]
            if(stopped_vel != 0):
                stopped_vel = 1

        f_wap = []
        if(self.astar == True):
            a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
            rx, ry = a_star.planning(sx, sy, gx, gy)
            for i in range(len(rx)):
                f_wap.append([rx[i], ry[i], 0])

        control = carla.VehicleControl()
        #print('------')
        #print(f_wap)
        #print('+++++')
        # if len(filtered_obstacles) > 0:
            #print(filtered_obstacles[0].get_location())

        if(self.current_lane == 0):
            lane = left_lane
        elif(self.current_lane == 1):
            lane = middle_lane
        else:
            lane = right_lane

        control.steer = self.stanley_controller2(f_wap, lane, transform, vel) #stanley controller
        control.throttle, control.brake = self.throttle_controller(boundary, transform, middle_lane, vel, filtered_obstacles, stopped_vel)
        #control.throttle = 0.7
        #print(f'Throttle: {control.throttle}')
        #print(np.sqrt(vel.x**2 + vel.y**2))
        return control


    def stanley_controller2(self, waypoints, g_waypoint, transform, vel):
        """
        Args:
        f_waypoint
            - Type:         List[x,y,z]
            - Description:  List of the x, y, z coordinate that we want the car to go to. 
        transform
            - Type:         carla.Transform 
            - Description:  Ego's current transform
        vel
            - Type:         carla.Vector3D 
            - Description:  Ego's current velocity in (x, y, z) in m/s
        """
        def normalize_angle(angle):
            while(angle < -np.pi):
                angle += 2*np.pi
            while(angle > np.pi):
                angle -= 2*np.pi
            return angle

        #getting all the required values for calculatons        
        cur_pos = transform.location
        current_x = cur_pos.x
        current_y = cur_pos.y
        current_z = cur_pos.z

        current_yaw = np.radians(transform.rotation.yaw)

        current_velocity = np.sqrt(vel.x**2 + vel.y**2)

        control_angle = 0

        used_waypoint = None
        dyi = None

        if(len(waypoints) > 6):
            dyi = 5
        elif(1 <= len(waypoints) <= 6):
            dyi = len(waypoints)-1
        else:
            dyi= -2 

        if(self.astar == False):
            dyi = -2

        #print("dyi: ", dyi)

        if dyi > 0:
            desired_yaw = np.arctan2((waypoints[0][1]-waypoints[dyi][1]), (waypoints[0][0]-waypoints[dyi][0]))
            used_waypoint = waypoints[dyi][1]

            #print("current yaw:", current_yaw)
            #print("desired_yaw:", desired_yaw)
            yaw_diff = desired_yaw - current_yaw
            yaw_diff = normalize_angle(yaw_diff)
            #print("yaw_diff:", yaw_diff)

            center_axle_current = np.array([current_x, current_y])
            crosstrack_error = np.min(np.sum((center_axle_current - np.array(waypoints)[:10, :2]) ** 2, axis=1))
            #print("Cross track error:", crosstrack_error)

            yaw_cross_track = np.arctan2(waypoints[0][1] - current_y, waypoints[0][0] - current_x)
            yaw_diff_of_path_cross_track = normalize_angle(desired_yaw - yaw_cross_track)
            crosstrack_error = -abs(crosstrack_error) if yaw_diff_of_path_cross_track > 0 else abs(crosstrack_error)
            #print("Yaw diff of path cross track:", yaw_diff_of_path_cross_track)

            k_e = 1
            k_v = 1

            yaw_diff_crosstrack = np.arctan(k_e * crosstrack_error / (k_v + current_velocity)) 
            #print("Yaw diff crosstrack:", yaw_diff_crosstrack)

            control_angle = max(-1.0, min(1.0, normalize_angle(yaw_diff + yaw_diff_crosstrack)))
            self.control_angle = control_angle
            #print("Control Angle:", control_angle)
        elif dyi == 0:
            desired_yaw = np.arctan2((current_y-waypoints[dyi][1]), (current_x-waypoints[dyi][0]))
            used_waypoint = waypoints[dyi][1]

            #print("current yaw:", current_yaw)
            #print("desired_yaw:", desired_yaw)
            yaw_diff = desired_yaw - current_yaw
            yaw_diff = normalize_angle(yaw_diff)
            #print("yaw_diff:", yaw_diff)

            center_axle_current = np.array([current_x, current_y])
            crosstrack_error = np.min(np.sum((center_axle_current - np.array(waypoints)[:10, :2]) ** 2, axis=1))
            #print("Cross track error:", crosstrack_error)

            yaw_cross_track = np.arctan2(waypoints[0][1] - current_y, waypoints[0][0] - current_x)
            yaw_diff_of_path_cross_track = normalize_angle(desired_yaw - yaw_cross_track)
            crosstrack_error = -abs(crosstrack_error) if yaw_diff_of_path_cross_track > 0 else abs(crosstrack_error)
            #print("Yaw diff of path cross track:", yaw_diff_of_path_cross_track)

            k_e = 1
            k_v = 1

            yaw_diff_crosstrack = np.arctan(k_e * crosstrack_error / (k_v + current_velocity)) 
            #print("Yaw diff crosstrack:", yaw_diff_crosstrack)

            control_angle = -max(-1.0, min(1.0, normalize_angle(yaw_diff + yaw_diff_crosstrack)))/5
            #print("******************************************************************************************************************************************************************************************************************************************************************************************************************************&")
            self.control_angle = control_angle
            #control_angle = self.control_angle

            #print("Control Angle:", control_angle)
        else:
            #print("Using default wp")
            desired_yaw = np.arctan2((g_waypoint[3][1]-g_waypoint[0][1]), (g_waypoint[3][0]-g_waypoint[0][0]))
            used_waypoint = g_waypoint[3][1]
            #print("current yaw:", current_yaw)
            #print("desired_yaw:", desired_yaw)
            yaw_diff = desired_yaw - current_yaw
            yaw_diff = normalize_angle(yaw_diff)
            #print("yaw_diff:", yaw_diff)

            center_axle_current = np.array([current_x, current_y])
            crosstrack_error = np.min(np.sum((center_axle_current - np.array(g_waypoint)[:10, :2]) ** 2, axis=1))
            #print("Cross track error:", crosstrack_error)

            yaw_cross_track = np.arctan2(g_waypoint[0][1] - current_y, g_waypoint[0][0] - current_x)
            yaw_diff_of_path_cross_track = normalize_angle(desired_yaw - yaw_cross_track)
            crosstrack_error = -abs(crosstrack_error) if yaw_diff_of_path_cross_track > 0 else abs(crosstrack_error)
            #print("Yaw diff of path cross track:", yaw_diff_of_path_cross_track)

            k_e = 1
            k_v = 1

            yaw_diff_crosstrack = np.arctan(k_e * crosstrack_error / (k_v + current_velocity)) 
            #print("Yaw diff crosstrack:", yaw_diff_crosstrack)

            control_angle = max(-1.0, min(1.0, normalize_angle(yaw_diff + yaw_diff_crosstrack)))
            self.control_angle = control_angle
            #print("Control Angle:", control_angle)

            #print("******************************************************************************************************************************************************************************************************************************************************************************************************************************&")

        return control_angle
    
    def throttle_controller(self, boundary, transform, future_waypoints, vel, obstacle_list, stopped_vel):
        """
        boundary 
            - Type:         List[List[left_boundry], List[right_boundry]]
            - Description:  left/right boundary each consists of 20 waypoints,
                            they defines the track boundary of the next 20 meters.

        transform
            - Type:         carla.Transform 
            - Description:  Ego's current transform
        """

        def pid_controller(self, Kp, Ki, Kd, T, desired_v, curr_v):
            error = desired_v - curr_v
            p = Kp * error
            self.integrator = self.integrator + (0.5 * Ki * T * (error + self.prev_error))
            u = p + self.integrator
            u = max(-1, min(1, u))
            if(u > 0):
                throttle = u
                brake = 0
            else:
                throttle = 0
                brake = abs(u)
            return throttle, brake
    
       

        def fit_circle(x1, y1, x2, y2, x3, y3):
            """Fits a circle to three points and returns the center and radius."""
            if x2 - x1 == 0 or y2 - y1 == 0:
                return 1000

            ma = (y2 - y1) / (x2 - x1)
            mb = (y3 - y2) / (x3 - x2)

            if mb-ma == 0:
                return 1000

            center_x = (ma*mb*(y1 - y3) + mb*(x1 + x2) - ma*(x2 + x3)) / (2*(mb - ma))
            center_y = (-1/ma)*(center_x - (x1 + x2)/2) + (y1 + y2)/2

            radius = np.sqrt((center_x - x1)**2 + (center_y - y1)**2)
            
            return radius

        # Get two points ahead in the waypoints
        desired_v = 0
        if len(future_waypoints) >= 50:
            
            radius1 = fit_circle(future_waypoints[0][0], future_waypoints[0][1], future_waypoints[5][0], future_waypoints[5][1], future_waypoints[10][0], future_waypoints[10][1])
            # #print("Radius 1: ", radius1)
            radius2 = fit_circle(future_waypoints[10][0], future_waypoints[10][1], future_waypoints[15][0], future_waypoints[15][1], future_waypoints[20][0], future_waypoints[20][1])
            # #print("Radius 2: ", radius2)
            radius3 = fit_circle(future_waypoints[20][0], future_waypoints[20][1], future_waypoints[25][0], future_waypoints[25][1], future_waypoints[30][0], future_waypoints[30][1])
            # #print("Radius 3: ", radius3)
            radius4 = fit_circle(future_waypoints[30][0], future_waypoints[30][1], future_waypoints[35][0], future_waypoints[35][1], future_waypoints[40][0], future_waypoints[40][1])
            # #print("Radius 4: ", radius4)
            radius5 = fit_circle(future_waypoints[40][0], future_waypoints[40][1], future_waypoints[45][0], future_waypoints[45][1], future_waypoints[49][0], future_waypoints[49][1])
            # #print("Radius 5: ", radius5)

            current_velocity = np.sqrt(vel.x**2 + vel.y**2)
            
            # Assuming a threshold radius of 50m to distinguish between straight and curve
            if radius5 < 30:
                desired_v5 = 18
            else:
                desired_v5 = 23

            if radius4 < 10:
                desired_v4 = 11
            elif radius4 < 15:
                desired_v4 = 13
            elif radius4 < 30:
                desired_v4 = 16
            elif radius4 < 55:
                desired_v4 = 21
            else:
                desired_v4 = 26

            if radius3 < 10:
                desired_v3 = 11
            elif radius3 < 15:
                desired_v3 = 13
            elif radius3 < 30:
                desired_v3 = 16
            elif radius3 < 55:
                desired_v3 = 16
            elif radius3 < 100:
                desired_v3 = 21
            else:
                desired_v3 = 26

            if radius2 < 10:
                desired_v2 = 11
            elif radius2 < 15:
                desired_v2 = 11
            elif radius2 < 30:
                desired_v2 = 16
            elif radius2 < 55:
                desired_v2 = 16
            elif radius2 < 100:
                desired_v2 = 21
            else:
                desired_v2 = 26

            if radius1 < 10:
                desired_v1 = 11
            elif radius1 < 15:
                desired_v1 = 13
            elif radius1 < 30:
                desired_v1 = 16
            elif radius1 < 55:
                desired_v1 = 16
            elif radius1 < 100:
                desired_v1 = 21
            else:
                desired_v1 = 26

            desired_v = min(desired_v1, desired_v2, desired_v3, desired_v4, desired_v5)
        else:
            desired_v = 26

        if(stopped_vel == 0):
            desired_v = 0
        elif(stopped_vel == 1):
            desired_v = 8
        
        #getting all the required values for calculatons        
        # cur_pos = transform.location
        # current_x = cur_pos.x
        # current_y = cur_pos.y
        # current_z = cur_pos.z

        # d_list = []
        # # a_list = []
        # if len(obstacle_list) > 0:
        #         for i in range(len(obstacle_list)):
        #             loc = obstacle_list[i].get_location()
        #             # angle = np.arctan2(current_y - loc.y, current_x - loc.x)
        #             dist = np.sqrt((current_x - loc.x)**2 + (current_y - loc.y)**2)
        #             d_list.append(dist)
        #             # a_list.append(angle)
        #         if 0 < min(d_list) < 5:
        #             desired_v = 0
        
       
        
                
        #desired_v = 15
        #print("Target Velocity: ", desired_v)

        control_throttle, control_brake = pid_controller(self, 2, 0.01, 1, 0.01, desired_v, current_velocity)

        # control_throttle = 0.6
        # control_brake = 0
    
        return control_throttle, control_brake

class AStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.resolution = resolution
        self.rr = rr
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.calc_obstacle_map(ox, oy)

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                #print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if current.x == goal_node.x and current.y == goal_node.y:
                #print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        #print("min_x:", self.min_x)
        #print("min_y:", self.min_y)
        #print("max_x:", self.max_x)
        #print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        #print("x_width:", self.x_width)
        #print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -2, math.sqrt(3)],
                  [-1, 2, math.sqrt(3)],
                  [1, -2, math.sqrt(3)],
                  [1, 2, math.sqrt(3)],
                  [-2, -1, math.sqrt(3)],
                  [-2, 1, math.sqrt(3)],
                  [2, -1, math.sqrt(3)],
                  [2, 1, math.sqrt(3)],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
