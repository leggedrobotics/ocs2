#ifndef IIT_BALLBOT_JOINT_DATA_MAP_H_
#define IIT_BALLBOT_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace Ballbot {

/**
 * A very simple container to associate a generic data item to each joint
 */
template<typename T> class JointDataMap {
private:
    T data[jointsCount];
public:
    JointDataMap() {};
    JointDataMap(const T& defaultValue);
    JointDataMap(const JointDataMap& rhs);
    JointDataMap& operator=(const JointDataMap& rhs);
    JointDataMap& operator=(const T& rhs);
          T& operator[](JointIdentifiers which);
    const T& operator[](JointIdentifiers which) const;
private:
    void copydata(const JointDataMap& rhs);
    void assigndata(const T& rhs);
};

template<typename T> inline
JointDataMap<T>::JointDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
JointDataMap<T>::JointDataMap(const JointDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const JointDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
JointDataMap<T>& JointDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& JointDataMap<T>::operator[](JointIdentifiers j) {
    return data[j];
}

template<typename T> inline
const T& JointDataMap<T>::operator[](JointIdentifiers j) const {
    return data[j];
}

template<typename T> inline
void JointDataMap<T>::copydata(const JointDataMap& rhs) {
    data[JBALL_X] = rhs[JBALL_X];
    data[JBALL_Y] = rhs[JBALL_Y];
    data[JBASE_Z] = rhs[JBASE_Z];
    data[JBASE_Y] = rhs[JBASE_Y];
    data[JBASE_X] = rhs[JBASE_X];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
    data[JBALL_X] = value;
    data[JBALL_Y] = value;
    data[JBASE_Z] = value;
    data[JBASE_Y] = value;
    data[JBASE_X] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
    out
    << "   jball_x = "
    << map[JBALL_X]
    << "   jball_y = "
    << map[JBALL_Y]
    << "   jbase_z = "
    << map[JBASE_Z]
    << "   jbase_y = "
    << map[JBASE_Y]
    << "   jbase_x = "
    << map[JBASE_X]
    ;
    return out;
}

}
}
#endif
