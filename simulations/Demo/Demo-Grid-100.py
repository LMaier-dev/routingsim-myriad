height = 10
width = 10

numn=[[0 for col in range(height+1)] for row in range(width+1)]
num=1
for x in range(1,width+1):
    for y in range(1,height+1):
        numn[x][y]=num
        num+=1

def edge(x1,y1,x2,y2):
    global numn
    print str(numn[x1][y1])+" "+str(numn[x2][y2])
    print str(numn[x2][y2])+" "+str(numn[x1][y1])

print "%i %i" % (height*width, 2*height*width - height - width)
for x in range(1,width+1):
    for y in range(1,height+1):
        if x<width: edge(x,y,x+1,y)
        if y<height: edge(x,y,x,y+1)

for x in range(1,width+1):
    for y in range(1,height+1):
        print "node "+str(numn[x][y])+' !ds:"p='+str(x*64)+','+str(y*64)+'"'

