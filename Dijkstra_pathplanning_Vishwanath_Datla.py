
# header files
import sys
import numpy as np
from heapq import heappush, heappop
import cv2

# class for Dijkstra
class Dijkstra(object):
    # init function
    def __init__(self, start, goal, clearance, radius):
        self.start = start
        self.goal = goal
        self.numRows = 250
        self.numCols = 400
        self.clearance = clearance
        self.radius = radius
        
    # move is valid 
    def IsValid(self, currRow, currCol):
        return (currRow >= (1 + self.radius + self.clearance) and currRow <= (self.numRows - self.radius - self.clearance) and currCol >= (1 + self.radius + self.clearance) and currCol <= (self.numCols - self.radius - self.clearance))

    # checks for an obstacle
    def IsObstacle(self, row, col):
        # constants
        sum_of_c_and_r = self.clearance + self.radius
        sqrt_of_c_and_r = 1.41 * sum_of_c_and_r
        
        # check circle
        dist1 = ((row - 185) * (row - 185) + (col - 300) * (col - 300)) - ((40 + sum_of_c_and_r) * (40 + sum_of_c_and_r))
        
       
        # check triangles
        (x1, y1) = (185,36)
        (x2, y2) = (180,80)
        (x3, y3) = (105,100)
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x1 - x3)) - ((y1 - y3) * (row - x3))
        dist3 = 1
        if(first >= 5 and second >= 5 and third >= 5):
            dist3 = 0
           
        (x1, y1) = (185,36)
        (x2, y2) = (210,115)
        (x3, y3) = (180,80)
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x1 - x3)) - ((y1 - y3) * (row - x3))
        dist4 = 1
        if(first >= 5 and second >= 5 and third >= 0):
            dist4 = 0
        
        (x1,y1) = (59.6,200)
        (x2,y2) = (79.8,235)
        (x3,y3) = (120.2,235)
        (x4,y4) = (140.4,200)
        (x5,y5) = (120.2,165)
        (x6,y6) = (79.8,165)
        first = ((col - y1) * (x2 - x1)) - ((y2 - y1) * (row - x1))
        '''
        second = ((col - y2) * (x3 - x2)) - ((y3 - y2) * (row - x2))
        third = ((col - y3) * (x4 - x3)) - ((y4 - y3) * (row - x3))
        fourth = ((col - y4) * (x5 - x4)) - ((y5 - y4) * (row - x4))
        fifth = ((col - y5) * (x6 - x5)) - ((y6 - y5) * (row - x5))
        sixth = ((col - y6) * (x1 - x6)) - ((y1 - y6) * (row - x6))
        '''
        second = ((col - y2) * (x6 - x2)) - ((y6 - y2) * (row - x2))
        third = ((col - y6) * (x1 - x6)) - ((y1 - y6) * (row - x6))
        dist5 = 1
        
        dist5 = 1
        if(first<=5 and second<=0 and third<=5):# and fourth<=5 and fifth<=5 and sixth>=5):
            dist5 = 0
        
        first = ((col - y2) * (x6 - x2)) - ((y6 - y2) * (row - x2))  #Horizontal line 1 
        second = ((col - y6) * (x5 - x6)) - ((y5 - y6) * (row - x6))
        third = ((col - y5) * (x3 - x5)) - ((y3 - y5) * (row - x5)) # horizontal line 2 
        fourth = ((col - y3) * (x2 - x3)) - ((y2 - y3) * (row - x3))
        dist6 = 1
        if(first>=0 and second>=0 and third>=5 and fourth>=5):
            dist6 = 0
        
        second = ((col - y5) * (x3 - x5)) - ((y3 - y5) * (row - x5))
        first = ((col - y4) * (x3 - x4)) - ((y3 - y4) * (row - x4))
        third = ((col - y4) * (x5 - x4)) - ((y5 - y4) * (row - x4))
        dist7 = 1
        if(first>=0 and second<=5 and third<=5):
            dist7 = 0
        if(dist1 <= 0  or dist3 == 0 or dist4==0 or dist5==0 or dist7==0 or dist6==0):
            return True
        return False

    # action move left
    def ActionMoveLeft(self, currRow, currCol):
        if(self.IsValid(currRow, currCol - 1) and self.IsObstacle(currRow, currCol - 1) == False):
            return True
        return False

    # action move right
    def ActionMoveRight(self, currRow, currCol):
        if(self.IsValid(currRow, currCol + 1) and self.IsObstacle(currRow, currCol + 1) == False):
            return True
        return False

    # action move up
    def ActionMoveUp(self, currRow, currCol):
        if(self.IsValid(currRow - 1, currCol) and self.IsObstacle(currRow - 1, currCol) == False):
            return True
        return False

    # action move down
    def ActionMoveDown(self, currRow, currCol):
        if(self.IsValid(currRow + 1, currCol) and self.IsObstacle(currRow + 1, currCol) == False):
            return True
        return False

    # action move right up
    def ActionMoveRightUp(self, currRow, currCol):
        if(self.IsValid(currRow - 1, currCol + 1) and self.IsObstacle(currRow - 1, currCol + 1) == False):
            return True
        return False

    # action move right down
    def ActionMoveRightDown(self, currRow, currCol):
        if(self.IsValid(currRow + 1, currCol + 1) and self.IsObstacle(currRow + 1, currCol + 1) == False):
            return True
        return False

    # action move left down
    def ActionMoveLeftDown(self, currRow, currCol):
        if(self.IsValid(currRow + 1, currCol - 1) and self.IsObstacle(currRow + 1, currCol - 1) == False):
            return True
        return False

    # action move left up
    def ActionMoveLeftUp(self, currRow, currCol):
        if(self.IsValid(currRow - 1, currCol - 1) and self.IsObstacle(currRow - 1, currCol - 1) == False):
            return True
        return False
    
    # dijkstra algorithm
    def Dijkstra(self):
        # create hashmap to store distances
        distMap = {}
        visited = {}
        path = {}
        for row in range(1, self.numRows + 1):
            for col in range(1, self.numCols + 1):
                distMap[(row, col)] = float('inf')
                path[(row, col)] = -1
                visited[(row, col)] = False
            
        # create queue, push the source and mark distance from source to source as zero
        explored_states = []
        queue = []
        heappush(queue, (0, self.start))
        distMap[self.start] = 0
    
        # run dijkstra algorithm and find shortest path
        while(len(queue) > 0):
            _, currNode = heappop(queue)
            visited[currNode] = True
            explored_states.append(currNode)
        
            # if goal node then exit
            if(currNode[0] == self.goal[0] and currNode[1] == self.goal[1]):
                break
        
            # go through each edge of current node
            if(self.ActionMoveLeft(currNode[0], currNode[1]) and visited[(currNode[0], currNode[1] - 1)] == False and (distMap[(currNode[0], currNode[1] - 1)] > distMap[currNode] + 1)):
                distMap[(currNode[0], currNode[1] - 1)] = distMap[currNode] + 1
                path[(currNode[0], currNode[1] - 1)] = currNode
                heappush(queue, (distMap[(currNode[0], currNode[1] - 1)], (currNode[0], currNode[1] - 1)))
            
            if(self.ActionMoveRight(currNode[0], currNode[1]) and visited[(currNode[0], currNode[1] + 1)] == False and (distMap[(currNode[0], currNode[1] + 1)] > distMap[currNode] + 1)):
                distMap[(currNode[0], currNode[1] + 1)] = distMap[currNode] + 1
                path[(currNode[0], currNode[1] + 1)] = currNode
                heappush(queue, (distMap[(currNode[0], currNode[1] + 1)], (currNode[0], currNode[1] + 1)))
            
            if(self.ActionMoveUp(currNode[0], currNode[1]) and visited[(currNode[0] - 1, currNode[1])] == False and (distMap[(currNode[0] - 1, currNode[1])] > distMap[currNode] + 1)):
                distMap[(currNode[0] - 1, currNode[1])] = distMap[currNode] + 1
                path[(currNode[0] - 1, currNode[1])] = currNode
                heappush(queue, (distMap[(currNode[0] - 1, currNode[1])], (currNode[0] - 1, currNode[1])))
            
            if(self.ActionMoveDown(currNode[0], currNode[1]) and visited[(currNode[0] + 1, currNode[1])] == False and (distMap[(currNode[0] + 1, currNode[1])] > distMap[currNode] + 1)):
                distMap[(currNode[0] + 1, currNode[1])] = distMap[currNode] + 1
                path[(currNode[0] + 1, currNode[1])] = currNode
                heappush(queue, (distMap[(currNode[0] + 1, currNode[1])], (currNode[0] + 1, currNode[1])))
            
            if(self.ActionMoveLeftDown(currNode[0], currNode[1]) and visited[(currNode[0] + 1, currNode[1] - 1)] == False and (distMap[(currNode[0] + 1, currNode[1] - 1)] > distMap[currNode] + 1.4142)):
                distMap[(currNode[0] + 1, currNode[1] - 1)] = distMap[currNode] + 1.4
                path[(currNode[0] + 1, currNode[1] - 1)] = currNode
                heappush(queue, (distMap[(currNode[0] + 1, currNode[1] - 1)], (currNode[0] + 1, currNode[1] - 1)))
            
            if(self.ActionMoveRightDown(currNode[0], currNode[1]) and visited[(currNode[0] + 1, currNode[1] + 1)] == False and (distMap[(currNode[0] + 1, currNode[1] + 1)] > distMap[currNode] + 1.4142)):
                distMap[(currNode[0] + 1, currNode[1] + 1)] = distMap[currNode] + 1.4
                path[(currNode[0] + 1, currNode[1] + 1)] = currNode
                heappush(queue, (distMap[(currNode[0] + 1, currNode[1] + 1)], (currNode[0] + 1, currNode[1] + 1)))
            
            if(self.ActionMoveRightUp(currNode[0], currNode[1]) and visited[(currNode[0] - 1, currNode[1] + 1)] == False and (distMap[(currNode[0] - 1, currNode[1] + 1)] > distMap[currNode] + 1.4142)):
                distMap[(currNode[0] - 1, currNode[1] + 1)] = distMap[currNode] + 1.4
                path[(currNode[0] - 1, currNode[1] + 1)] = currNode
                heappush(queue, (distMap[(currNode[0] - 1, currNode[1] + 1)], (currNode[0] - 1, currNode[1] + 1)))
            
            if(self.ActionMoveLeftUp(currNode[0], currNode[1]) and visited[(currNode[0] - 1, currNode[1] - 1)] == False and (distMap[(currNode[0] - 1, currNode[1] - 1)] > distMap[currNode] + 1.4142)):
                distMap[(currNode[0] - 1, currNode[1] - 1)] = distMap[currNode] + 1.4
                path[(currNode[0] - 1, currNode[1] - 1)] = currNode
                heappush(queue, (distMap[(currNode[0] - 1, currNode[1] - 1)], (currNode[0] - 1, currNode[1] - 1)))
        
        # return if no optimal path
        if(distMap[self.goal] == float('inf')):
            return (explored_states, [], distMap[self.goal])
        
        # backtrack path
        backtrack_states = []
        node = self.goal
        while(path[node] != -1):
            backtrack_states.append(node)
            node = path[node]
        backtrack_states.append(self.start)
        backtrack_states = list(reversed(backtrack_states))    
        return (explored_states, backtrack_states, distMap[self.goal])
    
    # animate path
    def animate(self, explored_states, backtrack_states, path):
        
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(str(path), fourcc, 20.0, (self.numCols, self.numRows))
        image = np.zeros((self.numRows, self.numCols, 3), dtype=np.uint8)
        count = 0
        for state in explored_states:
            image[int(self.numRows - state[0]), int(state[1] - 1)] = (255, 255, 0)
            if(count%75 == 0):
                out.write(image)
            count = count + 1

        count = 0
        for row in range(1, self.numRows + 1):
            for col in range(1, self.numCols + 1):
                if(image[int(self.numRows - row), int(col - 1), 0] == 0 and image[int(self.numRows - row), int(col - 1), 1] == 0 and image[int(self.numRows - row), int(col - 1), 2] == 0):
                    if(self.IsValid(row, col) and self.IsObstacle(row, col) == False):
                        image[int(self.numRows - row), int(col - 1)] = (154, 250, 0)
                        if(count%75 == 0):
                            out.write(image)
                        count = count + 1
            
        if(len(backtrack_states) > 0):
            for state in backtrack_states:
                image[int(self.numRows - state[0]), int(state[1] - 1)] = (0, 0, 255)
                out.write(image)
                cv2.imshow('result', image)
                cv2.waitKey(5)
                
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        out.release()

# take start and goal node as input
startRow = int(input("Enter the row coordinate for start node (between 1 and 250) : "))
startCol = int(input("Enter the column coordinate for start node (between 1 and 400) : "))
goalRow = int(input("Enter the row coordinate for goal node (between 1 and 250) : "))
goalCol = int(input("Enter the column coordinate for goal node (between 1 and 400) : "))

# define constants
start = (startRow, startCol)
goal = (goalRow, goalCol)
clearance = 0
radius = 0
dijkstra = Dijkstra(start, goal, clearance, radius)

if(dijkstra.IsValid(start[0], start[1])):
	if(dijkstra.IsValid(goal[0], goal[1])):
		if(dijkstra.IsObstacle(start[0],start[1]) == False):
			if(dijkstra.IsObstacle(goal[0], goal[1]) == False):
				(explored_states, backtrack_states, distance_from_start_to_goal) = dijkstra.Dijkstra()
				dijkstra.animate(explored_states, backtrack_states, "./dijkstra_point.avi")
				# print optimal path found or not
				if(distance_from_start_to_goal == float('inf')):
					print("\nNo optimal path found.")
				else:
					print("\nOptimal path found. Distance is " + str(distance_from_start_to_goal))
			else:
				print("The entered goal node is an obstacle ")
				print("Please check README.md file for running Dijkstra_point.py file.")
		else:
			print("The entered initial node is an obstacle ")
			print("Please check README.md file for running Dijkstra_point.py file.")
	else:
		print("The entered goal node outside the map ")
		print("Please check README.md file for running Dijkstra_point.py file.")
else:
	print("The entered initial node is outside the map ")
	print("Please check README.md file for running Dijkstra_point.py file.")
