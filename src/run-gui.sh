#!/bin/bash
make -j8 MODE=debug && ./RoutingSim_dbg ../simulations/r2kademlia/eval.ini
# Find objects in GUI:
# use the following expression in "Object full path filter"
# to find Nodes by their Overlay Key prefix (here 6927):
# *.protocol0.myOverlayKey AND info(6927*)
# Tick the checkbox "Watches, FSMs" (because OverlayKey is a registered Watch parameter)
# Search inside: simulation
# Class filter expression empty

