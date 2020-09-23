# Matt Kaplan - Modern Robotics, Course 4: Robot Motion Planning and Control
# Assignment 1

import numpy as np
import modern_robotics as mr
import csv

edges_filename = "edges.csv"
nodes_filename = "nodes.csv"

# Store node in a dict for easy lookup
nodes = {}
# read nodes file 
with open(nodes_filename, 'r') as csvfile:     
    csvreader = csv.reader(csvfile) 	        
    for row in csvreader: 
    	if row[0][0] != '#':    		
    		nodes[int(row[0])] = [int(row[0])-1, float(row[1]), float(row[2]), float(row[3])] 

# Build the edge cost matrix
cost = np.zeros((len(nodes),len(nodes)))
# read edges file 
with open(edges_filename, 'r') as csvfile:     
    csvreader = csv.reader(csvfile) 	        
    for row in csvreader: 
    	if row[0][0] != '#':    		
    		cost[int(row[1])-1][int(row[0])-1] = float(row[2])
    		cost[int(row[0])-1][int(row[1])-1] = float(row[2])

# Lists/Arrays to store A* data
past_cost = np.zeros(len(nodes))	# minimum cost so far for each tested node
OPEN = [0]							# Unexplored nodes
CLOSED = []							# Explored nodes
parent = [0]*len(nodes)				# Current shortest prior path to each node

# Keep going until there is nowhere left to explore
while len(OPEN) > 0:

	# Next traversed node
	current = OPEN.pop(0)

	# If we've found our way to the end
	if current == len(nodes)-1:
		print ("success")

		# Go backwards through the parent array
		i = len(nodes)-1
		out = []
		while i != 0:
			out.insert(0,i+1)
			i = parent[int(i)]
		out.insert(0,i+1)		
		print(out)

		# Dump path to csv
		filename = "path.csv"
		outfile = open(filename, 'w')
		csvwriter = csv.writer(outfile)  
		csvwriter.writerow(out)
		outfile.close()

		exit()

	# find neighbors from cost matrix
	neighbors = cost[current]

	# Main A* loop
	for i in range(len(neighbors)):
		neighbor = neighbors[i]

		# Check neighbors (non-zero) we haven't already explored
		if neighbor != 0 and not i in CLOSED:
			# New neighbor
			if past_cost[i] == 0.0:
				# Store path cost
				past_cost[i] = past_cost[current] + neighbor
				# Add to Unvisited list
				OPEN.append(i)  				
				j = len(OPEN)-1
				# Bubblesort down to place node path length in Unvisited list
				while j > 0 and past_cost[OPEN[j]] < past_cost[OPEN[j-1]]:
					OPEN[j], OPEN[j-1] = OPEN[j-1], OPEN[j]
				# Track parents for path recall later
				parent[i] = current
			# Seen this neighbor before
			else:
				# Check to see if current path is shorter than previous
				tenative_past_cost = past_cost[current] + neighbor
				if tenative_past_cost < past_cost[i]:
					# Store new, lower path cost
					past_cost[i] = tenative_past_cost
					# Find this neighbor in unvisited list
					j = 0
					while OPEN[j] != i:
						j += 1;					
					# Bubblesort down to place new node path length in Unvisited list
					while j > 0 and past_cost[OPEN[j]] < past_cost[OPEN[j-1]]:
						OPEN[j], OPEN[j-1] = OPEN[j-1], OPEN[j]
					# Update parents for path recall later
					parent[i] = current

	# We're done visiting this node
	CLOSED.append(current)

# If we've exited the loop, no solution was possible
print("no solution found")
