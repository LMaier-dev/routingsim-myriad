package routingsim.simulations.MYRIAD;

import routingsim.foundation.*;
import routingsim.dynamics.*;
import routingsim.mobility.*;

// Like DefaultNetwork but with NodeChurn: a steady trickle of single-node
// failures rather than LinkFailure's flaps or EscalatingFailure's waves. Used by
// eval_dynamics.py's "static" scenario, which measures the overhead floor under
// continuous background churn.
network ChurnNetwork
{
	submodules:
		manager: NetworkManager;
		dynamics: NodeChurn;
		mobilityDynamics: MobilityDynamics;
	connections:
		dynamics.toManager --> manager.dynamics;
		mobilityDynamics.toManager --> manager.mobilityDynamics;
}
