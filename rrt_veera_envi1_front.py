#!/usr/bin/env python

"""This code mainly implements RRT motion planning technique using pygame animation 
   as a part of course project for ROBOT MOTION PLANNING
   Developer : Veerendranath.Mannepalli
   Course : ITCS 6152
   PROFESSOR : Dr.Srinivas Akella
   ENVIRONMENT : MAP1 specified in Project report
   NON-HOLONOMIC : FRONT WHEEL CONSTRIANTS""" 
 

import sys, random, math, pygame
from pygame.locals import *
from math import *

#constants

EPSILON = 25.0
NUMNODES = 5000

a=0
b=0

#nodes list, tree dictionary and path specifiers
tree = {}
route=[]

obs_1 = []
obs_2 = []
obs_3 = []
obs_3 = []
obs_5 = []



#initialize and prepare screen
pygame.init()
screen = pygame.display.set_mode([1280,720])
pygame.display.set_caption('RRT NONHOLONOMIC RUNGEKUTTA FRONT WHEEL CONSTRAINTS ')
black = (  0,   0,   0)
white = (255, 255, 255)
red   = (255,   0,   0)
blue  = (0,   0,   255)
green  = (0,   255,   0)
fps = 60
screen.fill(white)
pygame.draw.rect(screen, black, [0, 100, 100, 400],1)
pygame.draw.rect(screen, black, [200, 0, 800, 150],1)
pygame.draw.rect(screen, black, [100, 300, 500, 25],1)
pygame.draw.rect(screen, black, [580, 425, 700, 25],1)
pygame.draw.rect(screen, black, [200, 550, 600, 50],1)
pygame.display.update()

#class for making sprites and building obstacles and robot
class Block(pygame.sprite.Sprite):
	
	def __init__(self,color,width,height):
		super(Block,self).__init__()
		self.image = pygame.Surface((width,height))
		self.image.fill(color)
		self.rect = self.image.get_rect()

	def set_position(self,x,y):
		self.rect.x = x
		self.rect.y = y


#Distance function 

def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))


#Runge-Kutta method for curve smoothing with non holonomic constraints
def runge_k_4(p1,steer):
    #print "entered"
    curve={}
    n=15
    h=EPSILON/n
    #print h
    curve_list=[]
    curve_list.append(p1)
    #print curve_list
    global obs_1
    global obs_2
    global obs_3
    global obs_4
    global obs_5
    
    for i in range(n):
	p=curve_list[-1]
        k1 = h*cos(steer)*cos(p[2]),h*cos(steer)*sin(p[2]),0.067*sin(steer)
	k2 = 0.5*k1[0]+h*cos(steer)*cos(p[2]),0.5*k1[1]+h*cos(steer)*sin(p[2]),0.5*k1[2]+0.067*sin(steer)
	k3 = 0.5*k2[0]+h*cos(steer)*cos(p[2]),0.5*k2[1]+h*cos(steer)*sin(p[2]),0.5*k2[2]+0.067*sin(steer)
	k4 = 0.5*k3[0]+h*cos(steer)*cos(p[2]),0.5*k3[1]+h*cos(steer)*sin(p[2]),0.5*k3[2]+0.067*sin(steer)
	newnode = p[0]+(k1[0]+k2[0]+k3[0]+k4[0])/6,p[1]+(k1[1]+k2[1]+k3[1]+k4[1])/6,p[2]+(k1[2]+k2[2]+k3[2]+k4[2])/6
	if point_in_poly(newnode[0],newnode[1],obs_1) and point_in_poly(newnode[0],newnode[1],obs_2) and point_in_poly(newnode[0],newnode[1],obs_3) and point_in_poly(newnode[0],newnode[1],obs_4) and point_in_poly(newnode[0],newnode[1],obs_5):
		curve_list.append(newnode)
		curve[newnode]={p:dist(p,newnode)}
	
        
	#print newnode
	
	
    return curve,newnode

def runge_k_4_draw(p1,steer):
    #print "entered"
    curve={}
    n=15
    h=EPSILON/n
    #print h
    curve_list=[]
    curve_list.append(p1)
    #print curve_list
    global obs_1
    global obs_2
    global obs_3
    global obs_4
    global obs_5
    
    for i in range(n):
	p=curve_list[-1]
        k1 = h*cos(steer)*cos(p[2]),h*cos(steer)*sin(p[2]),0.067*sin(steer)
	k2 = 0.5*k1[0]+h*cos(steer)*cos(p[2]),0.5*k1[1]+h*cos(steer)*sin(p[2]),0.5*k1[2]+0.067*sin(steer)
	k3 = 0.5*k2[0]+h*cos(steer)*cos(p[2]),0.5*k2[1]+h*cos(steer)*sin(p[2]),0.5*k2[2]+0.067*sin(steer)
	k4 = 0.5*k3[0]+h*cos(steer)*cos(p[2]),0.5*k3[1]+h*cos(steer)*sin(p[2]),0.5*k3[2]+0.067*sin(steer)
	newnode = p[0]+(k1[0]+k2[0]+k3[0]+k4[0])/6,p[1]+(k1[1]+k2[1]+k3[1]+k4[1])/6,p[2]+(k1[2]+k2[2]+k3[2]+k4[2])/6
	if point_in_poly(newnode[0],newnode[1],obs_1) and point_in_poly(newnode[0],newnode[1],obs_2) and point_in_poly(newnode[0],newnode[1],obs_3) and point_in_poly(newnode[0],newnode[1],obs_4) and point_in_poly(newnode[0],newnode[1],obs_5):
		curve_list.append(newnode)
		curve[newnode]={p:dist(p,newnode)}
		pygame.draw.line(screen,black,(p[0],p[1]),(newnode[0],newnode[1]))
        	pygame.display.update()
	
        
	#print newnode
	
	
    return curve,newnode
 
#Function to calculate configuration space aroud obstacles with robot configuration
def config_space(obstacle_vert,robotwidth,robotheight):
	obstacle_vert_conf = []
	
	x=0
	y=0
	li1= list(obstacle_vert[0])
	x=li1[0]
	y=li1[1]
	x=x-robotwidth
	y=y-robotheight
	li1[0]=x
	li1[1]=y
	tup1 = tuple(li1)
	obstacle_vert_conf.append(tup1)
	
	x=0
	y=0
	li1= list(obstacle_vert[1])
	x=li1[0]
	y=li1[1]
	x=x+robotwidth
	y=y-robotheight
	li1[0]=x
	li1[1]=y
	tup1 = tuple(li1)
	obstacle_vert_conf.append(tup1)
	
	x=0
	y=0
	li1= list(obstacle_vert[2])
	x=li1[0]
	y=li1[1]
	x=x+robotwidth
	y=y+robotheight
	li1[0]=x
	li1[1]=y
	tup1 = tuple(li1)
	obstacle_vert_conf.append(tup1)

	x=0
	y=0
	li1= list(obstacle_vert[3])
	x=li1[0]
	y=li1[1]
	x=x-robotwidth
	y=y+robotheight
	li1[0]=x
	li1[1]=y
	tup1 = tuple(li1)
	obstacle_vert_conf.append(tup1)

	return obstacle_vert_conf

# collision detection method using RAY CASTING Technique
def point_in_poly(x,y,poly):
    
    n = len(poly)
    outside = True
    global a
    global b 
    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xints:
                        outside = not outside
        p1x,p1y = p2x,p2y
	if outside:
		a=a+1
	else:
		b=b+1
    return outside
# RRT Planner for the Environment 
def main():
    full_group = pygame.sprite.Group()
    obstacle_group = pygame.sprite.Group()
    robot = Block(red,20,20)
    global obs_1
    global obs_2
    global obs_3
    global obs_4
    global obs_5
    
    
    robot.set_position(route[0][0],route[0][1])
    
    #print  robotwidth
    full_group.add(robot)
    full_group.draw(screen)
    obstacle1= Block(blue,100,400)
    obstacle1.set_position(0,100)
    obstacle1_vert = [obstacle1.rect.topleft,obstacle1.rect.topright,obstacle1.rect.bottomright,obstacle1.rect.bottomleft]
    obstacle1_config = config_space(obstacle1_vert,20,20)
    obs_1 = obstacle1_config
    print obstacle1_vert
    obstacle_group.add(obstacle1)
    full_group.add(obstacle1)
    #obstacle_group.draw(screen)

    obstacle2= Block(blue,800,150)
    obstacle2.set_position(200,0)
    obstacle2_vert = [obstacle2.rect.topleft,obstacle2.rect.topright,obstacle2.rect.bottomright,obstacle2.rect.bottomleft]
    obstacle2_config = config_space(obstacle2_vert,20,20)
    obs_2 = obstacle2_config
    print obstacle2_vert
    obstacle_group.add(obstacle2)
    full_group.add(obstacle2)
    #obstacle_group.draw(screen)

    obstacle3= Block(blue,500,25)
    obstacle3.set_position(100,300)
    obstacle3_vert = [obstacle3.rect.topleft,obstacle3.rect.topright,obstacle3.rect.bottomright,obstacle3.rect.bottomleft]
    obstacle3_config = config_space(obstacle3_vert,20,20)
    obs_3 = obstacle3_config
    print obstacle3_vert
    obstacle_group.add(obstacle3)
    full_group.add(obstacle3)

    obstacle4= Block(blue,700,25)
    obstacle4.set_position(580,425)
    obstacle4_vert = [obstacle4.rect.topleft,obstacle4.rect.topright,obstacle4.rect.bottomright,obstacle4.rect.bottomleft]
    obstacle4_config = config_space(obstacle4_vert,20,20)
    obs_4 = obstacle4_config
    print obstacle4_vert
    obstacle_group.add(obstacle4)
    full_group.add(obstacle4)

    obstacle5= Block(blue,600,50)
    obstacle5.set_position(200,550)
    obstacle5_vert = [obstacle5.rect.topleft,obstacle5.rect.topright,obstacle5.rect.bottomright,obstacle5.rect.bottomleft]
    obstacle5_config = config_space(obstacle5_vert,20,20)
    obs_5 = obstacle5_config
    print obstacle5_vert
    obstacle_group.add(obstacle5)
    full_group.add(obstacle5)

    pygame.display.update()
    nodes = []
    #start = (0,0)
    nodes.append(route[0]) # Start at start_node

    tree[route[0]]={None : 0}
    print(route[0])
    for i in range(NUMNODES):
	rand = random.random()*1280.0, random.random()*720.0, random.random()*360.0
	nn = nodes[0]
	#print 'entered'
        for p in nodes:							#loop to return previous node in tree
	   	if dist(p,rand) < dist(nn,rand):
			nn = p
	# Here it calculates the left ,right, straight points and determine which is near to the Random node
	curve1,newnode1 = runge_k_4(nn,math.radians(45.0))
	curve2,newnode2 = runge_k_4(nn,math.radians(0))
	curve3,newnode3 = runge_k_4(nn,math.radians(-45.0))
	if min(dist(rand,newnode1),dist(rand,newnode2),dist(rand,newnode3)) == dist(rand,newnode1):
		curve1,newnode1 = runge_k_4_draw(nn,math.radians(45.0))
		if point_in_poly(newnode1[0],newnode1[1],obstacle1_config) and point_in_poly(newnode1[0],newnode1[1],obstacle2_config) and point_in_poly(newnode1[0],newnode1[1],obstacle3_config) and point_in_poly(newnode1[0],newnode1[1],obstacle4_config) and point_in_poly(newnode1[0],newnode1[1],obstacle5_config):
			
			tree.update(curve1)
			nodes.append(newnode1)
			#print newnode1
		#print tree
	elif min(dist(rand,newnode1),dist(rand,newnode2),dist(rand,newnode3)) == dist(rand,newnode2):
		curve2,newnode2 = runge_k_4_draw(nn,math.radians(0))
		if point_in_poly(newnode2[0],newnode2[1],obstacle1_config) and point_in_poly(newnode2[0],newnode2[1],obstacle2_config) and point_in_poly(newnode2[0],newnode2[1],obstacle3_config) and point_in_poly(newnode2[0],newnode2[1],obstacle4_config) and point_in_poly(newnode2[0],newnode2[1],obstacle5_config):
			
			tree.update(curve2)
			nodes.append(newnode2)
			#print newnode2
		#print tree
	elif min(dist(rand,newnode1),dist(rand,newnode2),dist(rand,newnode3)) == dist(rand,newnode3):
		curve3,newnode3 = runge_k_4_draw(nn,math.radians(-45.0))
		if point_in_poly(newnode3[0],newnode3[1],obstacle1_config) and point_in_poly(newnode3[0],newnode3[1],obstacle2_config) and point_in_poly(newnode3[0],newnode3[1],obstacle3_config) and point_in_poly(newnode3[0],newnode3[1],obstacle4_config) and point_in_poly(newnode3[0],newnode3[1],obstacle5_config):
			
			tree.update(curve3)
			nodes.append(newnode3)
			#print newnode3
		#print tree
	
    print 'out of random loop'

      

    
    prede = tree.keys()   
    #print prede
    #Appending end node to the nodes list
    while route[1] not in prede:
	#print 'adding end_node'
	if point_in_poly(route[1][0],route[1][1],obstacle1_config) and point_in_poly(route[1][0],route[1][1],obstacle2_config) and point_in_poly(route[1][0],route[1][1],obstacle3_config) and point_in_poly(route[1][0],route[1][1],obstacle4_config) and point_in_poly(route[1][0],route[1][1],obstacle5_config):
		nn=prede[0]
		#print "entered"
		for p in prede:
			if dist(p,route[1]) < dist(nn,route[1]):
				if dist(p,route[1]) < 10:
					nodes.append(route[1])
					prede.append(route[1])
					print("End node is appended")
					newnode=route[1]
					tree[newnode]={p:dist(p,newnode)}
					pygame.draw.line(screen,red,(p[0],p[1]),(newnode[0],newnode[1]))
		        		pygame.display.update()
					break
	  			else:
					nn=p


    	
    print 'outside:',a
    print 'inside:',b
    print route
    global fps 
    predecessor = [route[1]]
    #here it draws the path from initial to goal nodes
    while predecessor[-1] != route[0]:
	pre=tree[predecessor[-1]]
	tup=pre.keys()
	pygame.draw.line(screen,blue,(predecessor[-1][0],predecessor[-1][1]),(tup[0][0],tup[0][1]))
	pygame.display.update()
	predecessor.append(tup[0])
	
        
    #print predecessor
    clock1 = pygame.time.Clock()
    length=len(predecessor)
    p = length-1
    while predecessor[p] != predecessor[0]:
	robot.set_position(predecessor[p][0],predecessor[p][1])
	
	clock1.tick(fps)
        full_group.draw(screen)
	pygame.display.flip()
	p=p-1
	#print p
	robot_hit = pygame.sprite.spritecollide(robot,obstacle_group,False)
	for block in robot_hit:
		print "collide"
		p=0
		break
    robot.set_position(predecessor[0][0],predecessor[0][1])
	
    clock1.tick(fps)
    full_group.draw(screen)
    pygame.display.flip()
    p=p-1
    #print p
    robot_hit = pygame.sprite.spritecollide(robot,obstacle_group,False)
    for block in robot_hit:
	print "collide"
	p=0
	break
        
        

   

if __name__ == '__main__':

    start_node=input('Enter start node :')
    route.append(start_node)
    end_node=input('Enter end node :')
    route.append(end_node)
    running = True		
    main()
    print "out of main finally"
    while(running):
	for event in pygame.event.get():
		if (event.type == pygame.QUIT):
			running = False
    pygame.quit()


input('press ENTER to exit')
