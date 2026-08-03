# Building

Rerun only when new directory in src (Protocol) or new .msg file or new libraries or smthg

```
source /opt/omnetpp-6.3.0/setenv

cd ../ && make makefiles && cd src
```

Rerun on .cc .h changes or for .ned .ini use touch
```
touch routing/MYRIAD/MyriadLs.cc && make -j$(nproc)
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
# Link State Variant
./RoutingSim -u Cmdenv ../simulations/MYRIAD/myriadls.ini -c PaperTopology
# Path Vector Variant
./RoutingSim -u Cmdenv ../simulations/MYRIAD/myriadgp.ini -c PaperTopology
```

## Ring memberships from a topology file

```
python3 topo2rings.py ../topologies/<file>.txt   # writes <file>.ringlabels.ini, <file>.png
```
- `<file>.ringlabels.ini` — per-node ringLabel lines (`include`d by the config)
- `<file>.png` — nodes colored by ring, multi-color = multi-ring (skipped >1000 nodes)
```
python3 topo2rings.py ../topologies/grid_10,10.txt
```

Prebuilt benchmark configs (`LargeTopology` = 50 nodes/6 rings) exist in both `.ini`s.
```
cd ../simulations && ../src/RoutingSim -u Cmdenv MYRIAD/myriadgp.ini -c LargeTopology
```