cplusplus {{
#include "Node_ID.h"
}}


//
// example message format
//

namespace routingsim;

class Node_ID {
 @existingClass;
}

packet StandardPkt {
     Node_ID via_Landmark;
     Node_ID destination;
     Node_ID source;
     Node_ID RoutingPath[];
     bool pastLandmark;
     int hops;
}
