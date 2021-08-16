import math
import matplotlib.pyplot as plt
import csv
import numpy as np

#settings:
angel= 15.0 #the amount of degrees in each section defult prob 20
points_file="data\\pointData0 (1)"


def points_in_section(points, degree1, degree2):
    #returns the points in the section from degree1 to degree2
    new_points= []
    e1= [1.0, 0.0]

    v1_cross= rotate(e1, degree1 + 180) 
    v2_cross= rotate(e1, degree2 + 180) 
    
    for point in points:
        if(np.dot(v1_cross, np.array(point))>0 and np.dot(v2_cross, np.array(point))<0):
            new_points.append(point)
    return new_points


def rotate(vector ,degree):
    #rotates vector by 'degree' degrees
    rotation = np.deg2rad(degree)
    rot = np.array([[math.cos(rotation), -math.sin(rotation)], [math.sin(rotation), math.cos(rotation)]])

    v = np.array(vector)
    new_v = np.dot(rot, v)
    return new_v



def main():
    max_value=0.0
    points= []
    x= []
    y= []
    with open(points_file+ ".csv", newline='') as f:
        reader = csv.reader(f)
        data = list(reader)
    for i in data:
        points.append([float(i[0]),float(i[2])]) #reads the points, z value= float(i[1])
        x.append(float(i[0]))
        y.append(float(i[2]))
        


    #TODO: maybe add clasters and noise cleaning



    #TODO: maybe only take points from some hight


    for section in range(0,360, 2): #checks each section 
        sub_points= points_in_section(points, section, section+angel) #all the points in the section
        distance=[]
        for point in sub_points:
            distance.append(math.sqrt(point[0]*point[0]+point[1]*point[1]))

        #TODO: add finding longest path of dots
        max=0
        max_index=0
        for i in range(len(distance)):
            if(distance[i]>max):
                max = distance[i]
                max_index = i
        value= max  #-np.amin(distance)

        if(value> max_value):
            max_value=value
            out_of_room=sub_points[max_index]

    plt.scatter(x,y,color= "red")
    plt.scatter([out_of_room[0]],[out_of_room[1]],color= "blue")

    plt.show() #prints the map, REMOVE WHEN IN DRONE

    #TODO: add a way to return the point but to the drone
    


        
main()

