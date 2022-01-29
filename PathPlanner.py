#!/usr/bin/env python

import math
from priority_queue import PriorityQueue


class PathPlanner:


    def __init__(self):
        """
        Class constructor
        """



    @staticmethod
    def grid_to_index(mapdata, x, y):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """

        ### REQUIRED CREDIT
        return y * mapdata.info.width + x

    @staticmethod
    def index_to_grid(mapdata, i):
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param x [int] The cell X coordinate.
        :param y [int] The cell Y coordinate.
        :return  [int] The index.
        """
        x = i%mapdata.info.width
        y = int(math.floor(i/mapdata.info.width))
        ### REQUIRED CREDIT
        return (x,y)


    @staticmethod
    def euclidean_distance(x1, y1, x2, y2):
        """
        Calculates the Euclidean distance between two points.
        :param x1 [int or float] X coordinate of first point.
        :param y1 [int or float] Y coordinate of first point.
        :param x2 [int or float] X coordinate of second point.
        :param y2 [int or float] Y coordinate of second point.
        :return   [float]        The distance.
        """

        ### REQUIRED CREDIT
        return math.sqrt((x1-x2)**2 + (y1-y2)**2)



    @staticmethod
    def grid_to_world(mapdata, x, y):
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The cell X coordinate.
        :param y       [int]           The cell Y coordinate.
        :return        [Point]         The position in the world.
        """

        ### REQUIRED CREDIT
        resolution = mapdata.info.resolution
        origin_x = mapdata.info.origin.position.x
        origin_y = mapdata.info.origin.position.y
        world_x = (x+0.5)*resolution + origin_x
        world_y = (y+0.5)*resolution + origin_y
        wc = Point()
        wc.x = world_x
        wc.y = world_y
        return wc



    @staticmethod
    def world_to_grid(mapdata, wp):
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """

        ### REQUIRED CREDIT
        resolution = mapdata.info.resolution
        origin_x = mapdata.info.origin.position.x
        origin_y = mapdata.info.origin.position.y
        grid_x = int((wp.x - origin_x)/resolution)
        grid_y = int((wp.y - origin_y)/resolution)
        return (grid_x, grid_y)



    @staticmethod
    def path_to_poses(mapdata, path):
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """

        ### REQUIRED CREDIT
        poseS = []
        counter = 0
        for cell in path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            point = PathPlanner.grid_to_world(mapdata, cell[0], cell[1])
            pose.pose.position = point

            Rot = 0
            if cell != path[-1]:
                nextCell = path[counter + 1]
                deltaX = nextCell[0] - cell[0]
                deltaY = nextCell[1] - cell[1]
                Rot = math.atan2(deltaY, deltaX)

            quaterninon = quaternion_from_euler(0, 0, Rot)
            pose.pose.orientation.x = quaterninon[0]
            pose.pose.orientation.y = quaterninon[1]
            pose.pose.orientation.z = quaterninon[2]
            pose.pose.orientation.w = quaterninon[3]

            counter += 1
            poseS.append(pose)
        # print(poseS)
        return poseS



    @staticmethod
    def is_cell_walkable(mapdata, x, y):
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [boolean]       True if the cell is walkable, False otherwise
        """

        ### REQUIRED CREDIT
        width = mapdata.info.width
        height = mapdata.info.height
        #print(x,y,width,height)
        #print(len(mapdata.data))
        #print(PathPlanner.grid_to_index(mapdata, x, y))
        if x >= width or x < 0 or y >= height or y < 0:
            return False
        if mapdata.data[PathPlanner.grid_to_index(mapdata, x, y)] > 50:
            return False
        return True



    @staticmethod
    def neighbors_of_4(mapdata, x, y):
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 4-neighbors.
        """

        ### REQUIRED CREDIT
        neighbors = []

        addx = [1, -1, 0, 0]
        addy = [0, 0, 1, -1]
        for i in range(len(addx)):
            if PathPlanner.is_cell_walkable(mapdata, x + addx[i], y + addy[i]):
                neighbors.append((x + addx[i], y + addy[i]))

        return neighbors



    @staticmethod
    def neighbors_of_8(mapdata, x, y):
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param x       [int]           The X coordinate in the grid.
        :param y       [int]           The Y coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """

        ### REQUIRED CREDIT
        neighbors = []

        addx = [1, -1, 0, 0, 1, 1, -1, -1]
        addy = [0, 0, 1, -1, 1, -1, -1, 1]
        for i in range(len(addx)):
            if PathPlanner.is_cell_walkable(mapdata, x + addx[i], y + addy[i]):
                neighbors.append((x + addx[i], y + addy[i]))
        return neighbors



    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Requesting the map")
        rospy.wait_for_service('/dynamic_map')
        try:
            GetMapData = rospy.ServiceProxy('/dynamic_map', GetMap)
            rospy.loginfo("Map obtained")
            return GetMapData().map
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def calc_cspace(self, mapdata, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Calculating C-Space")
        cSpaceMsg = GridCells()
        ## Go through each cell in the occupancy grid
        ## Inflate the obstacles where necessary
        tolerance = 50 # Defines what is a open and what is obstacle
        cspacemap = list(mapdata.data)
        # print(cspacemap)
        listOfObstacles = []
        for i in range(len(mapdata.data)):
            if(mapdata.data[i] > tolerance):
                listOfObstacles.append(i)
        for i in listOfObstacles:
            grid = PathPlanner.index_to_grid(mapdata, i)
            for j in range(padding+1):
                for k in range(padding+1):
                    if grid[0] + j < mapdata.info.width and grid[0] - j >= 0 and grid[1] + k < mapdata.info.height and grid[1] - k >= 0:
                        # Takes care of both sides of the grid decrasing to the center and fills each space with 100 to mark as Cspace
                        cspacemap[PathPlanner.grid_to_index(mapdata, grid[0] + j, grid[1] + k)] = 100
                        # print(grid[0], grid[1], PathPlanner.grid_to_index(mapdata, grid[0] + j, grid[1] + k))
                        cspacemap[PathPlanner.grid_to_index(mapdata, grid[0] + j, grid[1] - k)] = 100
                        cspacemap[PathPlanner.grid_to_index(mapdata, grid[0] - j, grid[1] + k)] = 100
                        cspacemap[PathPlanner.grid_to_index(mapdata, grid[0] - j, grid[1] - k)] = 100
                # This is a separate way to do Cspace which might take more time than above
                # for j in range((padding*2+1)**2):
                #     ychange = math.floor(j/(padding*2+1))*mapdata.info.width
                #     xchange = j%(padding*2+1)
                #     try:
                #         cspacemap[i+ychange+xchange-padding*mapdata.info.width-padding] = 100
                #     except:
                #         print("Index out of bounds")
        ## Create a GridCells message and publish it
        # print(cspacemap)
        Cells = []
        # Creates our GridCells message using a cells message and just copies ove the data from the cspace we gathered into the correct datatype
        for k in range(len(cspacemap)):
            if cspacemap[k] > tolerance:
                gridCoord = PathPlanner.index_to_grid(mapdata, k)
                Cells.append(PathPlanner.grid_to_world(mapdata, gridCoord[0], gridCoord[1]))
        # Makes header and cells message to publish as a GridCell
        cSpaceMsg.header.frame_id = "map"
        cSpaceMsg.header.stamp = rospy.Time.now()
        cSpaceMsg.cell_width = mapdata.info.resolution
        cSpaceMsg.cell_height = mapdata.info.resolution
        cSpaceMsg.cells = Cells
        self.pubC.publish(cSpaceMsg)
        ## Will return 3 numbers,
        ## Return the C-space
        rospy.loginfo("C-Space calculated")
        #print(cspacemap)

        finalCMap = mapdata
        finalCMap.data = tuple(cspacemap)
        return finalCMap



    def a_star(self, mapdata, start, goal):
        """The start and goal are a tuple in grid format, mappdata = Cspacedata"""
        ### REQUIRED CREDIT
        print("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        frontier = PriorityQueue()
        frontier.put(start, 0)
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        came_from[goal] = None

        # Create the frontiers
        frontierStuff = frontier.get_queue()
        frontierCells = []
        for priorityTuple in frontierStuff:
            gridTuple = priorityTuple[1]
            # print(gridTuple)
            frontierCells.append(PathPlanner.grid_to_world(mapdata, gridTuple[0], gridTuple[1]))


        #expanded cells to show pathed frontiers
        expandedCells = []


        #Go through graph until frontier is empty
        while not frontier.empty():
            current = frontier.get()

            # update the frontiers visited just to let us see visually if we want
            expandedCells.append(PathPlanner.grid_to_world(mapdata, current[0], current[1]))

            # If we have reached the goal, leave
            if current == goal:
                break

            #Add viable children to frontier
            for next in PathPlanner.neighbors_of_8(mapdata, current[0], current[1]):
                new_cost = cost_so_far[current] + PathPlanner.euclidean_distance(current[0], current[1], next[0], next[1])
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + PathPlanner.euclidean_distance(current[0], current[1], goal[0], goal[1])
                    frontier.put(next, priority)

                    # update frontier message
                    frontierStuff = frontier.get_queue()
                    frontierCells = []
                    for priorityTuple in frontierStuff:
                        gridTuple = priorityTuple[1]
                        frontierCells.append(PathPlanner.grid_to_world(mapdata, gridTuple[0], gridTuple[1]))
                    frontierMsg.cells = frontierCells
                    self.pubF.publish(frontierMsg)
                    rospy.sleep(0.1)

                    #add parent to came_from table
                    came_from[next] = current

        # Return the path found
        path = []
        current = goal
        if came_from[goal] != None:
            while came_from[current] != None:
                path.append(current)
                current = came_from[current]
            path.append(start)
        else:
            return path

        #reverse path to make it so the first element is the start
        path = path[::-1]
        rospy.loginfo("A* completed")
        #print(path)
        return path



    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        ### EXTRA CREDIT
        rospy.loginfo("Optimizing path")

        #Done by doing y = mx+b and checking if on line
        m = 1 #slope
        b = 0 #constant
        l = 1 #y slope
        t = 0 #y constant
        newPath = path[:]
        counter = 0
        for cell in path:
            if cell != path[-1]:
                nextCell = path[counter + 1]
                if nextCell[0] - cell[0] != 0:
                    if m * nextCell[0] + b != nextCell[1]:#if the slope has changed, make a new one!
                        m = (nextCell[1] - cell[1]) / (nextCell[0] - cell[0])
                        b = nextCell[1] - m * nextCell[0]
                    else:
                        if counter != 0: # since the optimizer starts at 0 and we want to make it as helpful as possible, this is necessary
                            newPath.remove(cell)# to keep the first position(start) as a node that has to be visited
                else:
                    if l * nextCell[1] + t != nextCell[0]:
                        l = (nextCell[0] - cell[0]) / (nextCell[1] - cell[1])
                        t = nextCell[0] - l * nextCell[1]
                    else:
                        if counter != 0:
                            newPath.remove(cell)
            counter += 1
            print(newPath)
        rospy.loginfo("Path Optimized")
        print(path)
        print("New Path is: ")
        print(newPath)
        return newPath




    def path_to_message(self, mapdata, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        ### REQUIRED CREDIT
        rospy.loginfo("Returning a Path message")
        thePath = Path()
        thePath.header.frame_id = "map"
        thePath.header.stamp = rospy.Time.now()
        thePath.poses = PathPlanner.path_to_poses(mapdata, path) #list of stamped poses

        return thePath




    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req
        """
        ## Request the map
        ## In case of error, return an empty path
        mapdata = PathPlanner.request_map()
        if mapdata is None:
            return Path()
        wheelbase = 0.16 #Wheelbase is 16cm
        ## Calculate the C-space and publish it
        cspacedata = self.calc_cspace(mapdata, int(math.ceil(wheelbase / 2 / mapdata.info.resolution))) #Inputs occupancy grid and necessary padding which is wheelbase / 2 / resolution
        ## Calculate frontier

        ## Execute A*
        start = PathPlanner.world_to_grid(mapdata, msg.start.pose.position)
        goal  = PathPlanner.world_to_grid(mapdata, msg.goal.pose.position)
        path  = self.a_star(cspacedata, start, goal)
        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        self.pubP.publish(self.path_to_message(mapdata, waypoints))
        ## Return a Path message
        return self.path_to_message(mapdata, waypoints)



    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.loginfo("STARTED PATH PLANNER")
        rospy.loginfo("WAITING FOR GETPLAN SRV")
        rospy.spin()



if __name__ == '__main__':
    PathPlanner().run()