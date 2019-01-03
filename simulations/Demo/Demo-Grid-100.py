numn=[[0 for col in range(11)] for row in range(11)]
num=1
for x in range(1,11):
    for y in range(1,11):
        numn[x][y]=num
        num+=1

def edge(x1,y1,x2,y2):
    global numn
    print str(numn[x1][y1])+" "+str(numn[x2][y2])
    print str(numn[x2][y2])+" "+str(numn[x1][y1])

print "100 100"
for x in range(1,11):
    for y in range(1,11):
        if x!=10: edge(x,y,x+1,y)
        if y!=10: edge(x,y,x,y+1)

for x in range(1,11):
    for y in range(1,11):
        print "node "+str(numn[x][y])+' !ds:"p='+str(x*64)+','+str(y*64)+'"'

