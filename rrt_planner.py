from controller import Supervisor
import numpy as np

TIME_STEP = 32

robot = Supervisor()  # create Supervisor instance

# case_node=robot.getFromDef('case')
# t_field_c = case_node.getField('translation')
# r_field_c = case_node.getField('rotation')

pshaft_node=robot.getFromDef('primaryshaft')
t_field_p = pshaft_node.getField('translation')
r_field_p = pshaft_node.getField('rotation')

sshaft_node=robot.getFromDef('secondaryshaft')
t_field_s = sshaft_node.getField('translation')
r_field_s = sshaft_node.getField('rotation')

case_node=robot.getFromDef('case')
t_field_c = case_node.getField('translation')
r_field_c = case_node.getField('rotation')

# [CODE PLACEHOLDER 1]
sfactor=3
count = 0

rad_o = 0.08
rad_s = 0.2

xmax=1
xmin=-5

zmax=8
zmin=2

amax=2.5
amin=-2.5

class node:
    def __init__(self):
        self.coord = []
        self.angle = 0
        self.parent = []
        self.child = []


def rrt(start, end, rad_s, rad_o):
    n_list = []
    n_list.append(start)
    prevcoord=start.coord
    prevangle=start.angle
    # n_list.append(end)
    iter=0
    endflag=False
    while endflag==False and iter<10000 and robot.step(TIME_STEP)!=-1:
        print(iter)
        iter+=1
        coord = np.array([np.random.uniform(xmin, xmax), 0, np.random.uniform(zmin, zmax)])
        angle=np.random.uniform(amin,amax)
        min_d=np.inf
        min_node=[]
        min_vec=[]
        for n in n_list: #find closest node and the dist to it
            diff_vec = n.coord-coord
            d=np.sum(np.square(diff_vec))
            if d<=min_d:
                min_d=d
                min_node=n
                min_vec=diff_vec
        if min_d>rad_s:
            min_vec=rad_s*min_vec/(np.sum(min_vec**2)**0.5)
            coord=min_node.coord+min_vec
        if abs(angle-min_node.angle)>rad_o:
            angle=min_node.angle+rad_o
        t_field_p.setSFVec3f(list(coord))
        r_field_p.setSFRotation([0,1,0,angle])
        pshaft_node.setVelocity([0,0,0,0,0,0])
        robot.step(TIME_STEP)
        
        if len(pshaft_node.getContactPoints())==0:
            prevcoord=coord.copy()
            prevangle=angle
            newnode=node()
            newnode.coord=coord
            newnode.angle=angle
            newnode.parent=min_node
            min_node.child.append(newnode)
            n_list.append(newnode)
            if np.sum(np.square(newnode.coord-end.coord))<rad_s and abs(end.angle-newnode.angle)<rad_o:
                newnode.child.append(end)
                end.parent=newnode
                endflag=True
                n_list.append(end)
                print('Found') 
        else:
            t_field_p.setSFVec3f(list(prevcoord))
            r_field_p.setSFRotation([0,1,0,prevangle])
            pshaft_node.setVelocity([0,0,0,0,0,0])
            #robot.step(TIME_STEP)
    return n_list       
        

        

start = node()
start.coord = np.array([-0.89, 0, 2.28])
start.angle = np.pi/2
start.parent = -1

end = node()
end.coord = np.array([-1.3, 0, 3.87])
end.angle = 2.08
end.child = -1

n_list = rrt(start, end, rad_s, rad_o)
#print(len(n_list))

coordlist=[]
rlist=[]

temp=n_list[-1]
coordlist.append(list(temp.coord))
rlist.append([0,1,0,temp.angle])

while temp.parent!=-1:
    coordlist.insert(0,list(temp.parent.coord))
    rlist.insert(0,[0,1,0,temp.parent.angle])
    temp=temp.parent

# for i in n_list:
    # coordlist.append(list(i.coord))
    # rlist.append([0,1,0,i.angle])
    
print(len(coordlist))
print(coordlist)
print("rlist: ",rlist)

count=0
while robot.step(TIME_STEP) != -1:
    # if i==1:
        # print(case_node.getOrientation())
    # if i==20:
        # r_field_c.setSFRotation([0,1,0,np.pi/2])
    # if i==40:
        # r_field_c.setSFRotation([0,1,0,np.pi/2])
        #t_field_c.setSFVec3f([1,0,0])
    # [CODE PLACEHOLDER 2]
    if count<len(rlist):
        t_field_p.setSFVec3f(coordlist[count])
        r_field_p.setSFRotation(rlist[count])
    count += 1