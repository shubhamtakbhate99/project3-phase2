
import cmath
import math
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
import numpy as np
import cv2
import matplotlib.pyplot as plt
import heapq as hq
import copy
# from include.problem import img
# from test import ppath







img = np.ones((1000,1000,3), np.uint8)
#Clearance
c=25
rec1 = np.array( [[[25-c,425-c],[175+c,425-c],[175+c,575+c],[25-c,575+c]]], dtype=np.int32 )
rec2= np.array( [[[375-c,425-c],[625+c,425-c],[625+c,575+c],[375-c,575+c]]], dtype=np.int32 )
rec3=np.array( [[[725-c,200-c],[875+c,200-c],[875+c,400+c],[725-c,400+c]]], dtype=np.int32 )


colors = (0,0,255)
cv2.fillPoly(img, rec1, colors )
cv2.fillPoly(img, rec2, colors )
cv2.fillPoly(img, rec3, colors)
cv2.circle(img,(200,800), 100+c,colors, -1)
cv2.circle(img,(200,200), 100+c,colors, -1)

display = img




class Node:

    
         
    def __init__(self, x, y, k):
        
        self.x = x
        self.y = y
        self.k = k
        self.g = 0
        self.f = 0
        self.h = 0
        self.set = 0
       

    def move(self,dis, k):
        
        self.x = round(self.x + dis*cmath.cos(np.pi*(k/6)).real)
        self.y = round(self.y + dis*cmath.sin(np.pi*(k/6)).real)
        
        self.k = (self.k + k)  

    
    def cost(self,UL,UR):
        t = 0
        r = 6.6
        L = 16
        dt = 0.1
        Xn= self.x
        Yn= self.y
        Thetan = 3.14 * self.k / 180
        D=0
        while t<1:
            t = t + dt
            
            Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
            Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
            Thetan += (r / L) * (UR - UL) * dt

        D=D+ math.sqrt(math.pow((0.5*r * (UL + UR) * math.cos(Thetan) * dt),2)+math.pow((0.5*r * (UL + UR) * math.sin(Thetan) * dt),2))
        Thetan = 180 * (Thetan) / 3.14
        return round(Xn), round(Yn), Thetan, D

    def plot(self,UL,UR,display,color):
            t = 0
            r = 6.6
            L = 16
            dt = 0.1
            Xn= self.x
            Yn= self.y
            Thetan = 3.14 * self.k / 180
        
            D=0
            while t<1:
                t = t + dt
                Xs = round(Xn)
                Ys = round(Yn)
                Xn += 0.5*r * (UL + UR) * math.cos(Thetan) * dt
                Yn += 0.5*r * (UL + UR) * math.sin(Thetan) * dt
                Thetan += (r / L) * (UR - UL) * dt
                display = cv2.line(display,(Xs,Ys),(round(Xn),round(Yn)),color,1)
        
        
    def edistance(self,node):
       return math.sqrt((self.x - node.x)**2 + (self.y - node.y)**2 )
   
    def check(self,visited):
        node = (self.x,self.y)
        
        if visited[node] == 1:
            return False
        else:
            return True
        
    def visit(self,visited):
        node = (self.x, self.y)
        visited[node] = 1
        
        
    def valid(self,visited,array):
        new = (self.x,self.y)
        pad = 5  # Padding of 5mm on boundering
        if  self.check(visited) and 0 + pad <= new[0] < 1000-pad and 0 + pad <= new[1] < 1000-pad and array[self.y,self.x] != 255 :

            return True
        else:
            
            return False
        
    def key(self):
        return (self.x,self.y,self.k)
        
        

def make_line(p1, p2):
    if p1[0]-p2[0] == 0:
        m = (p1[1]-p2[1])/0.00000001
    else:
        m = (p1[1]-p2[1])/(p1[0]-p2[0])
    c = -1*p1[0]*m + p1[1]
    return m, c

actions=[[5,5], [10,10],[5,0],[0,5],[5,10],[10,5],[10,0],[0,10]]
        
prob = display[:,:,2]

###########################################################################################

visited = np.zeros((1500,1500))
OpenList = {}

# x_s = int(input("Enter x start coordinate greater than 15 and less 385 : "))
# y_s = int(input("Enter y start coordinate  greater than 15 and less 235:"))
# theta_in=int(input("Enter intial theta angle:"))

start = Node(100, 100,0)

start.visit(visited)


goal = Node(900, 900,0)

# while not goal.valid(visited,prob):
#     print("Goal node inside object or out of limit. Please enter again")
#     goal.x = int(input("Enter x goal coordinate greater than 15 and less 385 : "))
#     goal.y = int(input("Enter y goal coordinate greater than 15 and less 235:"))
#     goal.k=int(input("Enter goal theta angle:"))

# s_t = int(input("enter step cost from 0 to 10 : "))
  

cv2.circle(display,(goal.x,goal.y),2,(0,255,0),-1)
OpenList[start.key()] = 0
node = copy.copy(start)
parent = {}

while OpenList:
    
    new = min(OpenList, key=OpenList.get)
    OpenList.pop(new)
    node  = Node(new[0],new[1],new[2])
    
    for action in actions:
        temp = copy.copy(node)
        x,y,th,d = temp.cost(action[0],action[1])
        
        temp = Node(x,y,th)

        dis = 0
        if temp.valid(visited,prob):
            if temp.edistance(goal)<=10+2:
                OpenList = {}
                parent[temp.key()] = node.key(),action
                break
                
            
            if temp.key() not in OpenList: 
                temp.g = node.g + d
                temp.h = temp.edistance(goal)
                temp.f = temp.g + temp.h
                OpenList[temp.key()] = temp.f 
                parent[temp.key()] = node.key(),action

                node.plot(action[0],action[1],display,(255,0,255))

            elif OpenList.get(temp.key()) > temp.f:
                templist = {temp.key():temp.f}
                parent.update({temp.key():node.key()})
                OpenList.update(templist)
                
            else:
                continue
        else:
            continue

        node.visit(visited)

        cv2.imwrite("/home/ameya/catkin_ws/src/search/src/display.png",np.flipud(display))
        
        
print("all Done")

path = []
ppath = []
ppath.append((goal.x,goal.y))

temp= node
while temp.key() != start.key():
    (x,y,k),action= parent.get(temp.key())
    temp = Node(x,y,k)
    path.append([temp.key(),action])
    ppath.append((x,y))



ppath.reverse()


for p in range(len(path)):

    (x,y,k),action = path[p]
    temp = Node(x,y,k)
    temp.plot(action[0],action[1],display,(0,255,0))

#cv2.imwrite(   path+"Visualise.png", np.flipud(display))




################################################################################




#cv2.imwrite(  path+"img.png",img)


#cv2.imwrite(  path+"Visualise.png", np.flipud(display))
x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
   
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


rospy.init_node("turtle")

sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

speed = Twist()
print(ppath)
r = rospy.Rate(4)

count = 0 
while not rospy.is_shutdown():
    
    for p in ppath:
        goal = Point()
        goal.x = p[0]/100 -5
        goal.y = p[1]/100 -5
        
        print("goal", (goal.x, goal.y))
        inc_x = 10
        inc_y = 10

        while cmath.sqrt(inc_x**2+inc_y**2).real > 0.2 :
            # inc_x = 0 
            # inc_y = 0
            inc_x = goal.x -x
            inc_y = goal.y -y
        #    print(inc_x,inc_y)
            print("goal", (goal.x, goal.y))
            angle_to_goal = math.atan(inc_y/(inc_x+ 1e-10))
            print("xy",x,y)
            print("theta",theta,angle_to_goal)

            speed.linear.x = 0.2
            if(inc_y > 0 and inc_x > 0) or (inc_y < 0 and inc_x > 0):
                speed.angular.z = 2*(angle_to_goal - theta)
            elif(inc_y < 0 and inc_x < 0) or (inc_y > 0 and inc_x < 0):
                speed.angular.z = 2*(angle_to_goal - theta + 3.14)
            
            pub.publish(speed)
            r.sleep()  

        count += 1
        if(len(ppath)-1 == count):
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            pub.publish(speed)
            exit()
