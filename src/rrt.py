import numpy as np
import matplotlib.pyplot as plt

class Env(object):
    class Obstacle(object):
        def __init__(self,o_x,o_y,radius):
            self.o_x = o_x
            self.o_y = o_y
            self.radius = radius

    def __init__(self,start_node,goal_node,obstacle_list=[],max_x=10,max_y=10,spatial_resolution=50):
        self.start_node = start_node
        self.goal_node = goal_node
        self.free_space_x = np.linspace(0,max_x,spatial_resolution)
        self.free_space_y = np.linspace(0,max_y,spatial_resolution)
        self.size = spatial_resolution
        self.obstacle_list = obstacle_list

    def get_random_point(self):
        mask = True
        while mask:
            rand_x = self.free_space_x[np.random.randint(self.size)]
            rand_y = self.free_space_y[np.random.randint(self.size)]
            node = Node(rand_x,rand_y)
            mask = self.check_colision(node)
        return node

    def check_colision(self,node):
        for obstacle in self.obstacle_list:
            dis = (node.x - obstacle.o_x)**2 + (node.y - obstacle.o_y)**2
            if dis < obstacle.radius**2 + 0.1:
                return True
        return False

    @staticmethod
    def get_min_distance_line_to_point(line,point):
        line = [a,b,c]
        point = x


class Node(object):
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0

    def __add__(self,node):
        return Node(self.x + node.x,self.y + node.y)

    def __radd__(self,node):
        return Node(node.x + self.x,node.y + self.y)

    def __sub__(self,node):
        return Node(self.x - node.x,self.y - node.y)

    def __rsub__(self,node):
        return Node(node.x - self.x,node.y - self.y)

    def __mul__(self,const):
        return Node(self.x*const,self.y*const)

    def __rmul__(self,const):
        return Node(self.x*const,self.y*const)

    def __pow__(self,const):
        return self.x**const + self.y**const

class Animation(object):
    def __init__(self,env):
        self.draw = None
        self.final_node = None
        plt.plot(env.start_node.x,env.start_node.y,'ko')
        plt.plot(env.goal_node.x,env.goal_node.y,'go')
        self.draw_obstacle(env.obstacle_list)

    def draw_final_path(self,final_node):
        node = final_node
        while node.parent != None:
            self.draw_line(node,node.parent,'r')
            plt.pause(0.01)
            node = node.parent
        return None

    def draw_obstacle(self,obstacle_list):
        for obstacle in obstacle_list:
            self.draw_circle(obstacle.o_x,obstacle.o_y,obstacle.radius)

    @staticmethod
    def draw_line(x_start,x_end,color):
        resolution = 50
        x = np.linspace(x_start.x,x_end.x,resolution)
        y = np.linspace(x_start.y,x_end.y,resolution)
        plt.plot(x,y,color)
        return None

    @staticmethod
    def draw_circle(x,y,radius):
        theta = np.linspace(0, 2*np.pi, 100)
        a = x + radius*np.cos(theta)
        b = y + radius*np.sin(theta)
        plt.plot(a, b)

class RRT(object):
    def __init__(self,env,ani):
        self.env = env
        self.ani = ani
        self.start_node = env.start_node
        self.goal_node = env.goal_node
        self.final_node = None
        self.node_list = [self.start_node]
        self.length = 0.4

    def rrt_planning(self):
        mask = not self.check_goal(self.start_node)
        while mask:
            colision_mask = False
            x_rand = self.env.get_random_point()
            x_closet = self.get_closet_point(x_rand,self.node_list)
            x_new_node = self.make_new_node(x_closet,x_rand)
            path_list = self.generate_path(x_closet,x_new_node)
            for path_node in path_list:
                if self.env.check_colision(path_node):
                    colision_mask = True
            if not colision_mask:
                self.ani.draw_line(x_closet,x_new_node,'b')
                plt.pause(0.01)
                self.node_list.append(x_new_node)
            mask = not self.check_goal(x_new_node)
        return self.final_node, self.node_list

    def check_goal(self,node):
        margin = 1
        dis = (node-self.goal_node)**2
        if dis < margin**2:
            self.final_node = node
            return True
        return False

    def make_new_node(self,x_start,x_end):
        delta = x_end - x_start
        s = np.sqrt(delta**2)

        if s != 0:
            delta *= self.length / s
            delta = delta + x_start
            delta.parent = x_start
            return delta
        return None

    @staticmethod
    def generate_path(first_node,second_node):
        resolution = 5
        delta = second_node-first_node
        dis = (delta)**2
        unit_vector = Node(delta.x/resolution,delta.y/resolution)
        path_list = [first_node + alpha*unit_vector for alpha in range(resolution)]
        return path_list

    @staticmethod
    def get_closet_point(point,node_list):
        dis_min = 100
        closet_point = [0,0]
        for node in node_list:
            dis = (node-point)**2
            if dis < dis_min:
                dis_min = dis
                closet_point_x = node.x
                closet_point_y = node.y
                closet_parent = node.parent
        closet_node = Node(closet_point_x,closet_point_y)
        closet_node.parent = closet_parent
        return closet_node

if __name__ == '__main__':
    start_node = Node(5,5)
    goal_node = Node(10,10)

    env = Env(start_node=start_node,goal_node=goal_node)
    env.obstacle_list = [env.Obstacle(1,1,1),env.Obstacle(6,7,0.3),env.Obstacle(8,8,1)]

    ani = Animation(env)
    rrt = RRT(env,ani)
    final_node, node_list = rrt.rrt_planning()
    ani.draw_final_path(final_node)
    plt.show()