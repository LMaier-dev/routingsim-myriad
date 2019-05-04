cplusplus {{
#include "foundation.h"
#include "DiscoPkt_m.h"
}}


namespace routingsim;
//
// example message format
//
class noncobject Node_ID;

packet StandardPkt {
	 Node_ID via_Landmark;
     Node_ID destination;
     Node_ID source;
     Node_ID RoutingPath[];
     bool pastLandmark;
     int hops;
}