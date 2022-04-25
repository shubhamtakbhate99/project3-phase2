import cmath
from cv2 import CAP_V4L
import numpy as np
import cv2
import matplotlib.pyplot as plt
import heapq as hq
import copy
import math




path = "/home/naveen/work661/src/search"
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
        if  self.check(visited) and 0 + pad <= new[0] < 1000-pad and 0 + pad <= new[1] < 1000-pad and array[self.y,self.x] > 0 :

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
        

prob = np.ones((1000, 1000))*np.exp(50)

pad = 5

pp1 = (36, 185)
pp2 = (115, 210)
pp3 = (80, 180)
pp4 = (105, 100)
ph = [(200, 100+70*np.tan(np.pi/6)), (235, 100+35/(2*np.cos(np.pi/6))),
      (235, 100-17.5/np.cos(np.pi/6)), (200, 100-70*np.tan(np.pi/6)),
      (165, 100-17.5/np.cos(np.pi/6)), (165, 100+17.5/np.cos(np.pi/6))]
for y in range(250):
    for x in range(400):
        m1, c1 = make_line(pp1, pp2)
        m2, c2 = make_line(pp2, pp3)
        m3, c3 = make_line(pp3, pp4)
        m4, c4 = make_line(pp4, pp1)
        if (y-m1*x < c1 and y-m4*x > c4) and (y-m2*x > c2 or y-m3*x < c3):
            prob[y, x] = 1
        mh1, ch1 = make_line(ph[0], ph[1])
        mh2, ch2 = make_line(ph[1], ph[2])
        mh3, ch3 = make_line(ph[2], ph[3])
        mh4, ch4 = make_line(ph[3], ph[4])
        mh5, ch5 = make_line(ph[4], ph[5])
        mh6, ch6 = make_line(ph[5], ph[0])
        if y-mh1*x < ch1 and y-mh2*x > ch2 and y-mh3*x > ch3 and y-mh4*x > ch4 and y-mh5*x > ch5 and y-mh6*x < ch6:
            prob[y, x] = 1

        xc, yc = 300, 185

        if np.sqrt((x-xc)**2 + (y-yc)**2) <= 40:
            prob[y, x] = 1
prob[prob == 1] = -1


display = np.zeros((prob.shape[0], prob.shape[1],3))
display[:, :, 2] = prob.copy()
display[:, :, 2][prob == -1] = 255  # Displaying Obstacles
display = display.astype(np.uint8)
###########################################################################################

visited = np.zeros((1500,1500))
OpenList = {}

# x_s = int(input("Enter x start coordinate greater than 15 and less 385 : "))
# y_s = int(input("Enter y start coordinate  greater than 15 and less 235:"))
# theta_in=int(input("Enter intial theta angle:"))

start = Node(100, 100,0)

while not start.valid(visited,prob):
    start.x = int(input("Enter x start coordinate greater than 15 and less 385 : "))
    start.y = int(input("Enter y start coordinate  greater than 15 and less 235:"))
    start.k =int(input("Enter intial theta angle:"))

start.visit(visited)

x_g = int(input("Enter x goal coordinate greater than 15 and less 385 : "))
y_g = int(input("Enter y goal coordinate greater than 15 and less 235:"))
theta_goal=int(input("Enter goal theta angle:"))

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
        
        cv2.imshow("Visualise!", np.flipud(display))
        cv2.waitKey(1)
print("all Done")

path = []
ppath = []

temp= node
while temp.key() != start.key():
    (x,y,k),action= parent.get(temp.key())
    temp = Node(x,y,k)
    path.append([temp.key(),action])
    ppath.append((x,y))
ppath = ppath.reverse()


for p in range(len(path)):

    (x,y,k),action = path[p]
    temp = Node(x,y,k)
    temp.plot(action[0],action[1],display,(0,255,0))

cv2.imwrite("Visualise.png", np.flipud(display))