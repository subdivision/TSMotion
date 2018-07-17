#!/usr/bin/python
import argparse
import random

cubes_unit_size = float(1.5)
directions = [(0,-1),(1,0),(0,1),(-1,0)]

def set_start(mat, edges):
	mat[1][2] = True
	mat[2][1] = True
	mat[2][2] = True
	mat[2][3] = True
	edges[0].append((2,1))
	edges[1].append((2,3))

def print_robot_file(startRobot1, startRobot2, endRobot1, endRobot2, fp):
	fp.write("{} {} {} {}\n".format(startRobot1[0],startRobot1[1],endRobot1[0],endRobot1[1]))
	fp.write("{} {} {} {}\n".format(startRobot2[0],startRobot2[1],endRobot2[0],endRobot2[1]))
	fp.close()

def print_robot_file1(edges, fp):
	while True:
		x = random.sample(edges, 2)
		y = random.sample(edges, 2)
		if x[0] != y[0] and x[1] != y[1]:
			print_robot_file(to_xy(x[0]),to_xy(y[0]),to_xy(x[1]),to_xy(y[1]),fp)
			return
	
	
def is_legal(mat,point):
	if point[0]==0 or point[1]==0 or point[0]==len(mat)-1 or point[1]==len(mat[0])-1:
		return False
	count = 0
	for direction in directions:
		if mat[point[0]+direction[0]][point[1]+direction[1]]:
			count+=1
	if count != 1:
		return False
	count = 0
	countLegel = 0
	for i in range(-1,2):
		for j in range(-1,2):
			if i != 0 and j != 0:
				if mat[point[0]+i][point[1]+j]:
					if mat[point[0]+i][point[1]] or mat[point[0]][point[1]+j]:
						count+=1
					else:
						return False
	return count <= 1		
	
def handle_edge(mat, point):
	points = []
	for direction in directions:
		p1 = (point[0]+direction[0], point[1]+direction[1])
		if is_legal(mat, p1):
			points.append(p1)
	if len(points) == 0:
		return []
	
	points = random.sample(points, random.randrange(len(points))+1)
	for p1 in points:
		mat[p1[0]][p1[1]] = True
	
	return points
			
def generate(mat,edges):
	while len(edges[0])!=0 or len(edges[1])!=0:
		i = random.randrange(2)
		if len(edges[i]) != 0:
			e = edges[i].pop(random.randrange(len(edges[i])))
			e_to_add = handle_edge(mat, e)
			if len(e_to_add) == 0:
				edges[2] += [e]
			else:
				edges[i] += e_to_add


def to_xy(p):
	x = p[1]*cubes_unit_size
	y = p[0]*cubes_unit_size
	
	if len(p)!=3:
		x+= (cubes_unit_size-1)/2
		y+= (cubes_unit_size-1)/2
	elif p[2] == 1:
		y+=cubes_unit_size
	elif p[2] == 2:
		y+=cubes_unit_size
		x+=cubes_unit_size
	elif p[2] == 3:
		x+=cubes_unit_size
	return (x,y)

def print_poly(poly,fp):
	s = ""
	last = None
	n = 0
	for p in poly:
		if p != last:
			n += 1
			s += " {} {}".format(p[0],p[1])
			last = p
	fp.write("{}{}\n0\n".format(n,s))
	
def print_mat(mat, fp):
	polys = []
	start = [1,2,0]
	x = start[:]
	polys += [to_xy(start)]
	x[2] = 1
	while x!= start:
		polys += [to_xy(x)]
		y = [x[0] + directions[x[2]][0], x[1] + directions[x[2]][1], (x[2] + (4-1)) % 4]
		if mat[y[0]][y[1]]:
			x = y
		else:
			x[2] = (x[2] + 1) % 4
	print_poly(polys, fp)
	fp.close()
		

# Main function
if __name__ == '__main__':
	parser = argparse.ArgumentParser(description='Generate a polygon.')
	parser.add_argument('n', type=int, default=5, help='number of rows')
	parser.add_argument('m', type=int, default=5, help='number of collumns')
	parser.add_argument('obstaclesFileName', help='file to save the obstacles to')
	parser.add_argument('robotFileName', help='file to save the robot to')

	args = parser.parse_args()
	n = args.n
	m = args.m
	obstaclesFileName = args.obstaclesFileName
	robotFileName = args.robotFileName

	if (n < 5 or m < 5):
		parser.error('Both n and m must be greater then 5 ({}x{})'.format(n,m))

	mat = []
	for _ in range(n):
		mat.append([False]*m)
	edges = [[],[],[]]
	set_start(mat,edges)
	generate(mat,edges)
	print_mat(mat, open(obstaclesFileName,'w'))
	print_robot_file1(edges[2], open(robotFileName,'w'))
