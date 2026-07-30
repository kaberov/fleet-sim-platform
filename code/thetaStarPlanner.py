"""

Theta* grid planning

author: Musab Kasbati (@Musab1Blaser)

See Wikipedia article (https://cdn.aaai.org/AAAI/2007/AAAI07-187.pdf)

"""

import math
import xml.etree.ElementTree as ET
import json


def build_map(world_path_file, working_zone):

    ox = []
    oy = []

    for i in range(int(working_zone[0][0]), int(working_zone[1][0]) + 1):
        ox.append(float(i))
        oy.append(working_zone[0][1])
    for i in range(int(working_zone[0][0]), int(working_zone[1][0]) + 1):
        ox.append(float(i))
        oy.append(working_zone[1][1])
    for i in range(int(working_zone[0][1]), int(working_zone[1][1]) + 1):
        ox.append(working_zone[0][0])
        oy.append(float(i))
    for i in range(int(working_zone[0][1]), int(working_zone[1][1]) + 1):
        ox.append(working_zone[1][0])
        oy.append(float(i))

    tree = ET.parse(world_path_file)
    root = tree.getroot()

    for model in root.findall(".//model"):
        model_name = model.get('name')
        pose = model.find('pose')
        if pose is not None:
            # print(f"Model: {model_name}, Pose: {pose.text}")
            ox.append(float(pose.text.split(' ')[0]))
            oy.append(float(pose.text.split(' ')[1]))

    return ox, oy


class ThetaStarPlanner:

    def __init__(self, ox, oy, resolution, rr):
        """
        Initialize grid map for theta star planning

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
        Theta star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        snaped_sx = round(sx / self.resolution) * self.resolution
        snaped_sy = round(sy / self.resolution) * self.resolution
        snaped_gx = round(gx / self.resolution) * self.resolution
        snaped_gy = round(gy / self.resolution) * self.resolution

        start_node = self.Node(self.calc_xy_index(snaped_sx, self.min_x),
                               self.calc_xy_index(snaped_sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(snaped_gx, self.min_x),
                              self.calc_xy_index(snaped_gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
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
                                    current.cost + self.motion[i][2], c_id)  # cost may later be updated by theta star path compression
                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                # Theta* modification:
                if current.parent_index != -1 and current.parent_index in closed_set:
                    grandparent = closed_set[current.parent_index]
                    if self.line_of_sight(grandparent, node):
                        # If parent(current) has line of sight to neighbor
                        node.cost = grandparent.cost + math.hypot(node.x - grandparent.x, node.y - grandparent.y)
                        node.parent_index = current.parent_index # compress path directly to grandparent

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node 


        rx, ry = self.calc_final_path(goal_node, closed_set)

        rx.reverse()
        ry.reverse()

        rx[0] = sx
        ry[0] = sy

        rx[-1] = gx
        ry[-1] = gy

        # rx = [sx] + rx + [gx]
        # ry = [sy] + ry + [gy]

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

    def line_of_sight(self, node1, node2):
        """
        Check if there is a direct line of sight between two nodes.
        Uses Bresenham’s line algorithm for grid traversal.
        """
        x0 = node1.x
        y0 = node1.y
        x1 = node2.x
        y1 = node2.y

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy

        while True:
            if not self.verify_node(self.Node(x0, y0, 0, -1)):
                return False
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return True


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

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

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
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main():
    print(__file__ + " start!!")

    with open('config.json', 'r') as file:
        data = json.load(file)

    # start and goal position
    sx = 43.0
    sy = -4.0
    gx = 23.0
    gy = -3.0
    grid_size = 2.0 
    robot_radius = data["robot_radius"]

    ox, oy = build_map(data["world_name"], data["working_zone"])

    theta_star = ThetaStarPlanner(ox, oy, grid_size, robot_radius)

    rx, ry = theta_star.planning(sx, sy, gx, gy)

    print(rx, ry)


if __name__ == '__main__':
    main()