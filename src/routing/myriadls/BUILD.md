




 
# Building

Rerun only when new directory in src (Protocol) or new .msg file or new libraries or smthg

```
source /opt/omnetpp-6.3.0/setenv

cd ../ && make makefiles && cd src
```

Rerun on .cc .h changes or for .ned .ini use touch
```
touch routing/myriadls/MyriadLs.cc && make -j$(nproc)
```

# Running

## Paper Topo

Ring A: `{0,1,3}`  Ring B: `{1,2,3,4,6}`  Ring C: `{2,5,6}`
```
-A---B---C-
   1---2
  /|   |\
 0 |   | 5
  \|   |/
   3-4-6
```
Run within `/src` directory
```
./RoutingSim -u Cmdenv ../simulations/myriadls/MyriadLs.ini -c PaperTopology
```