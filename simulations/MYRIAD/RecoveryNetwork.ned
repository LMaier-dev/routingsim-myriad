package routingsim.simulations.MYRIAD;

import routingsim.foundation.*;
import routingsim.dynamics.*;
import routingsim.mobility.*;

// Like DefaultNetwork but with node failures (EscalatingFailure: one run,
// growing waves) instead of LinkFailure, for eval_dynamics.py. Kept here rather
// than swapping DefaultNetwork's dynamics so the shared network is untouched.
network RecoveryNetwork
{
	submodules:
		manager: NetworkManager;
		dynamics: EscalatingFailure;
		mobilityDynamics: MobilityDynamics;
	connections:
		dynamics.toManager --> manager.dynamics;
		mobilityDynamics.toManager --> manager.mobilityDynamics;
}
