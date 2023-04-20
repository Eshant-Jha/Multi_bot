import search
import time
import copy
import maze_maps
import matplotlib
import matplotlib.pyplot as plt

class Maze:
  """
  This class outlines the structure of the maze problem
  """
  
  maze_map = []# To store map data, start and goal points
  
  # Legal moves
  # [delta_x, delta_y, description]
  five_neighbor_actions = {'up':[-1, 0], 'down':[1, 0], 'left': [0, -1], 'right': [0, 1], 'stop': [0, 0]}
  #eight_neighbor_actions = {'up':[-1, 0], 'down':[1, 0], 'left': [0, -1], 'right': [0, 1], 'stop': [0, 0], 
  #                        'upright':[-1, 1], 'upleft':[-1, -1], 'downright':[1, 1], 'downleft':[1, -1]}
  
  #Setup plot
  map_plot_copy = []
  plot_colormap_norm = matplotlib.colors.Normalize(vmin=0.0, vmax=29.0)
  fig,ax = plt.subplots(1)
  plt.axis('equal')

  def plot_map(self):
      """
      Plot
      """
      #Plotting robot 1
      start = self.getStartState(1)
      goal = self.getGoalState(1)
      self.map_plot_copy[start[0]][start[1]] = maze_maps.r1_start_id
      self.map_plot_copy[goal[0]][goal[1]] = maze_maps.r1_goal_id
      #plotting robot 2
      start = self.getStartState(2)
      goal = self.getGoalState(2)
      self.map_plot_copy[start[0]][start[1]] = maze_maps.r2_start_id
      self.map_plot_copy[goal[0]][goal[1]] = maze_maps.r2_goal_id
      #plotting robot 3
      
      start = self.getStartState(3)
      goal = self.getGoalState(3)
      self.map_plot_copy[start[0]][start[1]] = maze_maps.r3_start_id
      self.map_plot_copy[goal[0]][goal[1]] = maze_maps.r3_goal_id
      
      plt.imshow(self.map_plot_copy, cmap=plt.cm.tab20c, norm=self.plot_colormap_norm)
      plt.show()
      
  # default constructor
  def __init__(self, id):
      """
      Sets the map as defined in file maze_maps
      """
      #Set up the map to be used
      self.maze_map = maze_maps.maps_dictionary[id]
      self.map_plot_copy = copy.deepcopy(self.maze_map.map_data)
      self.plot_map()
      return
     
  def getStartState(self, robot_id):
     """
     Returns the start state for the search problem 
     """
     if robot_id == 1:
         start_state = self.maze_map.r1_start 
     elif robot_id== 2:
        start_state = self.maze_map.r2_start
     else: 
         start_state =self.maze_map.r3_start
     return start_state
 
  def getGoalState(self, robot_id):
     """
     Returns the start state for the search problem 
     """
     if robot_id == 1:
         goal_state = self.maze_map.r1_goal
     elif robot_id ==2:
         goal_state = self.maze_map.r2_goal
     else:
         goal_state = self.maze_map.r3_goal
     return goal_state
    
  def isGoalState(self, robot_id, state):
     """
       state: Search state
    
     Returns True if and only if the state is a valid goal state
     """
     if state[0:2] == self.getGoalState(robot_id):
         return True
     else:
         return False

  def isObstacle(self, state):
      """
        state: Search state
     
      Returns True if and only if the state is an obstacle
      """
      if self.maze_map.map_data[state[0]][state[1]] == maze_maps.obstacle_id:
          return True
      else:
          return False
      
  def getSuccessors(self, state, dynamic_obstacle1, dynamic_obstacle2):
 
     """
       state: Seacrh state
     
     For a given state, this should return a list of triples, 
     (successor, action, stepCost), where 'successor' is a 
     successor to the current state, 'action' is the action
     required to get there, and 'stepCost' is the incremental 
     cost of expanding to that successor
     """
     successors = []
     for action in self.five_neighbor_actions:
         
         #Get individual action
         del_x, del_y = self.five_neighbor_actions.get(action) 
         
         #Get successor
         new_successor = [state[0] + del_x , state[1] + del_y, state[2]+1]
         new_action = action
         
         # Check for static obstacle 
         if self.isObstacle(new_successor):
             continue
         
         if dynamic_obstacle1:
             #Check for dynamic obstacle 
             if (abs(new_successor[0] - dynamic_obstacle1[0]) + abs(new_successor[1] - dynamic_obstacle1[1])) < 2:
                 continue
             #Check for dynamic obstacle 
             
         if  dynamic_obstacle2:
             if (abs(new_successor[0] - dynamic_obstacle2[0]) + abs(new_successor[1] - dynamic_obstacle2[1])) < 2:
                 continue
             
         
         #cost
         new_cost = maze_maps.free_space_cost 
             
         successors.append([new_successor, new_action, new_cost])
         
     return successors

 

       
