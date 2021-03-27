#ifndef IIT_WHEELS_CHIMERA_JOINT_DATA_MAP_H_
#define IIT_WHEELS_CHIMERA_JOINT_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace wheels_chimera {

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
    data[LF_HAA] = rhs[LF_HAA];
    data[LF_HFE] = rhs[LF_HFE];
    data[LF_KFE] = rhs[LF_KFE];
    data[LF_WHEEL] = rhs[LF_WHEEL];
    data[RF_HAA] = rhs[RF_HAA];
    data[RF_HFE] = rhs[RF_HFE];
    data[RF_KFE] = rhs[RF_KFE];
    data[RF_WHEEL] = rhs[RF_WHEEL];
    data[LH_HAA] = rhs[LH_HAA];
    data[LH_HFE] = rhs[LH_HFE];
    data[LH_KFE] = rhs[LH_KFE];
    data[LH_WHEEL] = rhs[LH_WHEEL];
    data[RH_HAA] = rhs[RH_HAA];
    data[RH_HFE] = rhs[RH_HFE];
    data[RH_KFE] = rhs[RH_KFE];
    data[RH_WHEEL] = rhs[RH_WHEEL];
}

template<typename T> inline
void JointDataMap<T>::assigndata(const T& value) {
    data[LF_HAA] = value;
    data[LF_HFE] = value;
    data[LF_KFE] = value;
    data[LF_WHEEL] = value;
    data[RF_HAA] = value;
    data[RF_HFE] = value;
    data[RF_KFE] = value;
    data[RF_WHEEL] = value;
    data[LH_HAA] = value;
    data[LH_HFE] = value;
    data[LH_KFE] = value;
    data[LH_WHEEL] = value;
    data[RH_HAA] = value;
    data[RH_HFE] = value;
    data[RH_KFE] = value;
    data[RH_WHEEL] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const JointDataMap<T>& map) {
    out
    << "   LF_HAA = "
    << map[LF_HAA]
    << "   LF_HFE = "
    << map[LF_HFE]
    << "   LF_KFE = "
    << map[LF_KFE]
    << "   LF_WHEEL = "
    << map[LF_WHEEL]
    << "   RF_HAA = "
    << map[RF_HAA]
    << "   RF_HFE = "
    << map[RF_HFE]
    << "   RF_KFE = "
    << map[RF_KFE]
    << "   RF_WHEEL = "
    << map[RF_WHEEL]
    << "   LH_HAA = "
    << map[LH_HAA]
    << "   LH_HFE = "
    << map[LH_HFE]
    << "   LH_KFE = "
    << map[LH_KFE]
    << "   LH_WHEEL = "
    << map[LH_WHEEL]
    << "   RH_HAA = "
    << map[RH_HAA]
    << "   RH_HFE = "
    << map[RH_HFE]
    << "   RH_KFE = "
    << map[RH_KFE]
    << "   RH_WHEEL = "
    << map[RH_WHEEL]
    ;
    return out;
}

}
}
#endif
