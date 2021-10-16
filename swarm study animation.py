import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

'return distance between points (x1,y1) and (x2,y2)'

def distance(x1,y1,x2,y2):
    
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    
'return x ^ n or 0 when x = 0 and n < 0'

def power(x,n):
    
    return np.where(x == 0, 0, x ** n)
    
'return cosine of angle between vectors (x1,y1) and (x2,y2)'
    
def cosine(x1,y1,x2,y2):
    
    mag_product = (x1 ** 2 + y1 ** 2) ** 0.5 * (x2 ** 2 + y2 ** 2) ** 0.5
        
    return np.where(mag_product == 0, 1, (x1 * x2 + y1 * y2) / mag_product)

'return component of vector joining points (x1,y1) and (x2,y2), along specified axis'

def component(x1,y1,x2,y2,axis):
    
    if axis == 'x':
        
        return (x1 - x2) / distance(x1,y1,x2,y2)
    
    if axis == 'y':
        
        return (y1 - y2) / distance(x1,y1,x2,y2)

N = 30 #number of agents
T = 1000 #number of time steps

X = np.asarray([np.random.normal(0,100) for i in range(N)]) #initial x positions of agents
Y = np.asarray([np.random.normal(0,100) for i in range(N)]) #initial y positions of agents
V_X = np.asarray([np.random.normal(0,10) for i in range(N)]) #initial x velocities of agents
V_Y = np.asarray([np.random.normal(0,10) for i in range(N)]) #initial y velocities of agents

x_paths = [[] for i in range(N)] #x_paths[j][k] will be x position of agent i at time step j
y_paths = [[] for i in range(N)] #y_paths[j][k] will be y position of agent i at time step j

'parameters for agent behaviour'

attract_weight_power = np.random.uniform(-3,3)
repel_weight_power = np.random.uniform(-3,3)
align_weight_power = np.random.uniform(-3,3)
align_cos_coef = np.random.uniform(0,1)
attract_strength = np.random.uniform(-10,0)
attract_power = np.random.uniform(0,1)
repel_strength = np.random.uniform(0,10)
repel_power = np.random.uniform(-3,0)
align_strength = np.random.uniform(0,1)
transform_strength = np.random.uniform(0,1)
transform_coef = np.random.uniform(-1,0)

'at each time step, acceleration of each agent will be calculated'
'position of each agent will be updated by adding current velocity to current position, velocity updated by adding acceleration to current velocity'

for t in range(T):
        
    'set centre of mass of swarm to (0,0) - I am only interested in motion of agents relative to it'
    
    X -= np.average(X)
    Y -= np.average(Y)
    
    print(t)
    
    'arrays of x and y accelerations, element k will be x / y acceleration of agent k' 
        
    A_X = np.zeros(N)
    A_Y = np.zeros(N)
    
    'x and y accelerations each the sum of 3 parts: attraction to neighbours, repulsion from neighbours, and alignment with velocities of neighbours'
    'will calculate these separately then add appropriately to A_X and A_Y'
    
    A_X_attract = np.zeros(N)
    A_Y_attract = np.zeros(N)
    
    A_X_repel = np.zeros(N)
    A_Y_repel = np.zeros(N)
    
    A_X_align = np.zeros(N)
    A_Y_align = np.zeros(N)
    
    'calculating each of the N elements of the above 6 parts, corresponding to the N agents'
    
    for i in range(N):
        
        'x and y positions and velocities of agent i'
        
        X_i = X[i]
        Y_i = Y[i]
        V_X_i = V_X[i]
        V_Y_i = V_Y[i]
        
        'append current x and y positions of agent i to ends of its x and y paths'
        
        x_paths[i].append(X_i)
        y_paths[i].append(Y_i)
        
        'the following will allow for further calculations...'
                    
        X_i_vector = X_i * np.ones(N)
        Y_i_vector = Y_i * np.ones(N)
        V_X_i_vector = V_X_i * np.ones(N)
        V_Y_i_vector = V_Y_i * np.ones(N)
        
        '...i.e. these'
        
        D_ij = distance(X_i_vector,Y_i_vector,X,Y) #element j is the distance between agent i and agent j
        D_X_ij = X - X_i_vector #element j is the x displacement from agent i to agent j
        D_Y_ij = Y - Y_i_vector #element j is the y displacement from agent i to agent j
        
        C_ij = cosine(D_X_ij,D_Y_ij,V_X_i_vector,V_Y_i_vector) #element j is the cosine of the angle between agent i's direction of motion and the vector pointing from agent i to agent j
        
        'the attraction and repulsion parts of the acceleration of agent i are calculated as follows:'
        'a weighted average position of all agents aside from agent i is calculated'
        'the acceleration part is then a function of the position of agent i and this weighted average position'
        'for the alignment part, replace position with velocity in the above two lines'
        
        'element i in below weightings is zero as per definition of power function'
        
        W_attract_ij = power(D_ij,attract_weight_power) #element j is attraction weighting of agent j, attract_weight_power < 0 so agent i cares more about closer agents
        W_repel_ij = power(D_ij,repel_weight_power) #element j is repulsion weighting of agent j, repel_weight_power < 0 so agent i cares more about closer agents
        W_align_ij = power(D_ij,align_weight_power) * np.exp(align_cos_coef * C_ij) #element j is alignment weighting of agent j, repel_weight_power < 0 so agent i cares more about closer agents, align_cos_coef > 0 so agent i cares more about agents in its peripheral vision
        
        'x and y weighted average positions and velocities'
        
        X_i_attract = np.sum(X * W_attract_ij) / np.sum(W_attract_ij)
        Y_i_attract = np.sum(Y * W_attract_ij) / np.sum(W_attract_ij)       
        X_i_repel = np.sum(X * W_repel_ij) / np.sum(W_repel_ij)
        Y_i_repel = np.sum(Y * W_repel_ij) / np.sum(W_repel_ij)
        V_X_i_align = np.sum(V_X * W_align_ij) / np.sum(W_align_ij)
        V_Y_i_align = np.sum(V_Y * W_align_ij) / np.sum(W_align_ij)
        
        'for use in calculation of attraction and repulsion parts of acceleration of agent i'
        
        D_i_attract = distance(X_i,Y_i,X_i_attract,Y_i_attract)
        D_i_repel = distance(X_i,Y_i,X_i_repel,Y_i_repel)
        
        'calculation of each part of acceleration of agent i'
        
        A_X_attract[i] += attract_strength * component(X_i,Y_i,X_i_attract,Y_i_attract,'x') * D_i_attract ** attract_power
        A_Y_attract[i] += attract_strength * component(X_i,Y_i,X_i_attract,Y_i_attract,'y') * D_i_attract ** attract_power
        
        A_X_repel[i] += repel_strength * component(X_i,Y_i,X_i_repel,Y_i_repel,'x') * D_i_repel ** repel_power
        A_Y_repel[i] += repel_strength * component(X_i,Y_i,X_i_repel,Y_i_repel,'y') * D_i_repel ** repel_power
    
        A_X_align[i] += align_strength * (V_X_i_align - V_X_i)
        A_Y_align[i] += align_strength * (V_Y_i_align - V_Y_i)
        
    'adding all relevant parts to x and y components of accelarations of each agent'
        
    A_X += A_X_attract + A_X_repel + A_X_align 
    A_Y += A_Y_attract + A_Y_repel + A_Y_align
    
    A = (A_X ** 2 + A_Y ** 2) ** 0.5 #acceleration magnitudes of each agent
    
    'limiting maximum x and y components of accelerations by multiplying by logistic function of acceleration magnitudes and dividing by acceleration magnitudes'
    
    A_X *= transform_strength * (1 - np.exp(transform_coef * A)) / A 
    A_Y *= transform_strength * (1 - np.exp(transform_coef * A)) / A
    
    'updating agent positions and velocities'
    
    X += V_X
    Y += V_Y
    
    V_X += A_X 
    V_Y += A_Y 

x_min = min([x for X in x_paths for x in X])
y_min = min([y for Y in y_paths for y in Y])
x_max = max([x for X in x_paths for x in X])
y_max = max([y for Y in y_paths for y in Y])

fig,axes = plt.subplots()
axes.axis([x_min,x_max,y_min,y_max])

points = []

for i in range(N):
    
    points.append(axes.plot(0,0,marker='.')[0])
  
def animate(i):
    
    for j in range(N):
                
        points[j].set_data([x_paths[j][i]],[y_paths[j][i]])
        
    return points

anim = animation.FuncAnimation(fig,animate,frames=[i+1 for i in range(T-1)], interval=10, blit=True, repeat=True)

plt.show()

