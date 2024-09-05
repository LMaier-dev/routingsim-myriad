#ifndef Node_ID_H
#define Node_ID_H

#include "BaseTypes.h"

namespace routingsim {

class Node_ID {
public:
        nodeid_t id;
        const char *origin= nullptr;
        bool operator==(const Node_ID &cmp) const {
                return(id==cmp.id);
        }
};

inline std::ostream& operator<<(std::ostream& os, const Node_ID& u) {
    os << " id=" << u.id << " origin=" << (u.origin != nullptr ? u.origin : "NULL") ;
    return os;
}

} // end namespace

#endif // Node_ID_H guard
