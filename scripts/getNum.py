#!/usr/bin/env python
fileName = "test.log"
f = open(fileName, 'r')
f2 = open("trans_vel.txt", 'w')
f3 = open("rot_vel.txt", 'w')
f4 = open("d_dist.txt", 'w') 
f5 = open("d_rvel.txt", 'w')
f6 = open("d_tvel.txt", 'w')
f2.write("[")
f3.write("[")
f4.write("[")
f5.write("[")
f6.write("[")
stVrstic = 0
data = []
for line in f:
    stolpec = 0
    item = "a"
    for x in line.split(' '):
            if stolpec == 0:
                item = x
                stolpec +=1
            elif stolpec == 1:
                f2.write(str(x.strip()) + ", ")
                stolpec +=1
            elif stolpec == 2:
                #if not(float(x) == 0):
                    #print item + " " + str(x)
                f3.write(str(x.strip()) + ", ")
                stolpec +=1
            elif stolpec == 3:
                #if not(float(x) == 0):
                    #print item + " " + str(x)
                f4.write(str(x.strip()) + ", ")
                stolpec +=1
            elif stolpec == 4:
                #if not(float(x) == 0):
                    #print item + " " + str(x)
                f5.write(str(x.strip()) + ", ")
                stolpec +=1
            elif stolpec == 5:
                #if not(float(x) == 0):
                    #print item + " " + str(x)
                f6.write(str(x.strip()) + ", ")
                stolpec +=1
    stVrstic +=1
f2.write("]")
f3.write("]")
f4.write("]")
f5.write("]")
f6.write("]")
f.close()
f2.close()
f3.close()
f4.close()
f5.close()
f6.close()


