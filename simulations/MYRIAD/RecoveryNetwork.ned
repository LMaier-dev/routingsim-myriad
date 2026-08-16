package routingsim.simulations.MYRIAD;

import routingsim.foundation.*;
import routingsim.dynamics.*;
import routingsim.mobility.*;

// Like DefaultNetwork but with node failures (CatastrophicFailure) instead of
// LinkFailure, for the recovery eval (eval_recovery.py). Kept here rather than
// swapping DefaultNetwork's dynamics so the shared network is untouched.
network RecoveryNetwork
{
	submodules:
		manager: NetworkManager;
		dynamics: CatastrophicFailure;
		mobilityDynamics: MobilityDynamics;
	connections:
		dynamics.toManager --> manager.dynamics;
		mobilityDynamics.toManager --> manager.mobilityDynamics;
}
