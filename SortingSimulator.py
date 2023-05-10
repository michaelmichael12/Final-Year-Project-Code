import matplotlib.pyplot as plt
import numpy as np
import random


cardlist = [1,1,1,1,2,2,2,2,3,3,3,3,4,4,4,4,5,5,5,5,6,6,6,6,7,7,7,7,8,8,8,8,9,9,9,9,10,10,10,10,11,11,11,11,12,12,12,12,13,13,13,13]
random.shuffle(cardlist)
sortedlist = [0]*52
x = np.arange(0,52,1)
xsorted = np.arange(0,52,1)
n = len(cardlist)

def bubble():
     for i in range(n):
        for j in range(0, n-i-1):
            plt.bar(x,cardlist)
            plt.pause(0.00001)
            plt.clf()
            if cardlist[j] > cardlist[j+1]:
                cardlist[j], cardlist[j+1] = cardlist[j+1], cardlist[j]

def selection():
    num = 1
    for ind in range(0,52,4):
        count = 0
        for position in range(ind, 52):#starts position 1, then 5 then ...
            plt.bar(x,cardlist)
            plt.pause(0.00001)
            plt.clf()
            if cardlist[position] == num:
                    count = count + 1
                    if count == 1:
                        cardlist[ind], cardlist[position] = cardlist[position], cardlist[ind]
                    elif count == 2:
                        cardlist[ind+1], cardlist[position] = cardlist[position], cardlist[ind+1]
                    elif count == 3:
                       
                        cardlist[ind+2], cardlist[position] = cardlist[position], cardlist[ind+2]
                    elif count == 4:
                        cardlist[ind+3], cardlist[position] = cardlist[position], cardlist[ind+3]
        num = num+1
        


def selectionupgraded():
    num = 1
    for ind in range(0,52,12):
        #print("index = ",ind)
        #print("number we r looking for = ",num)
        #print("--------------------------------")
        count = 0
        count2 = 0
        count3 = 0
        for position in range(0, 52):#starts position 0, then 4 then ...
            plt.bar(x+0.2, cardlist, color = 'b', width = 0.4, label = 'unsorted')
            plt.bar(x-0.2, sortedlist, color = 'g',width = 0.4, label = 'sorted')
            plt.pause(0.00001)
            plt.clf()
            if cardlist[position] == num:
                    count = count + 1
                    if count == 1:
                        sortedlist[ind], cardlist[position] = cardlist[position], 0
                    elif count == 2:
                        sortedlist[ind+1], cardlist[position] = cardlist[position], 0
                    elif count == 3:
                        sortedlist[ind+2], cardlist[position] = cardlist[position], 0
                    elif count == 4:
                        sortedlist[ind+3], cardlist[position] = cardlist[position], 0
                    
            elif cardlist[position] == (num+1):
                    count2 = count2 + 1
                    if count2 == 1:
                        sortedlist[ind+4], cardlist[position] = cardlist[position], 0
                    elif count2 == 2:
                        sortedlist[ind+5], cardlist[position] = cardlist[position], 0
                    elif count2 == 3:
                        sortedlist[ind+6], cardlist[position] = cardlist[position], 0
                    elif count2 == 4:
                        sortedlist[ind+7], cardlist[position] = cardlist[position], 0
            elif cardlist[position] == (num+2):
                    count3 = count3 + 1
                    if count3 == 1:
                        sortedlist[ind+8], cardlist[position] = cardlist[position], 0
                    elif count3 == 2:
                        sortedlist[ind+9], cardlist[position] = cardlist[position], 0
                    elif count3 == 3:
                        sortedlist[ind+10], cardlist[position] = cardlist[position], 0
                    elif count3 == 4:
                        sortedlist[ind+11], cardlist[position] = cardlist[position], 0   
        num = num+3

     

#selection()
#bubble()    
selectionupgraded()