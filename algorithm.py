import math
import matplotlib.pyplot as plt
import csv

def main():
    d= []
    x= []
    y= []
    with open('pointData0.csv', newline='') as f:
        reader = csv.reader(f)
        data = list(reader)
    for i in data:
        d.append([i[0],i[2],i[1]])
        x.append(i[0])
        y.append(i[2])
    print(d)
    

    plt.scatter(x,y)
    plt.show()

        
main()

