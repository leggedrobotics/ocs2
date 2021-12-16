#ifndef IIT_CAMEL_LINK_DATA_MAP_H_
#define IIT_CAMEL_LINK_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace camel {

/**
 * A very simple container to associate a generic data item to each link
 */
template<typename T> class LinkDataMap {
private:
    T data[linksCount];
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LinkDataMap() {};
    LinkDataMap(const T& defaultValue);
    LinkDataMap(const LinkDataMap& rhs);
    LinkDataMap& operator=(const LinkDataMap& rhs);
    LinkDataMap& operator=(const T& rhs);
          T& operator[](LinkIdentifiers which);
    const T& operator[](LinkIdentifiers which) const;
private:
    void copydata(const LinkDataMap& rhs);
    void assigndata(const T& commonValue);
};

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const T& value) {
    assigndata(value);
}

template<typename T> inline
LinkDataMap<T>::LinkDataMap(const LinkDataMap& rhs)
{
    copydata(rhs);
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const LinkDataMap& rhs)
{
    if(&rhs != this) {
        copydata(rhs);
    }
    return *this;
}

template<typename T> inline
LinkDataMap<T>& LinkDataMap<T>::operator=(const T& value)
{
    assigndata(value);
    return *this;
}

template<typename T> inline
T& LinkDataMap<T>::operator[](LinkIdentifiers l) {
    return data[l];
}

template<typename T> inline
const T& LinkDataMap<T>::operator[](LinkIdentifiers l) const {
    return data[l];
}

template<typename T> inline
void LinkDataMap<T>::copydata(const LinkDataMap& rhs) {
    data[BASE] = rhs[BASE];
    data[LF_HIP] = rhs[LF_HIP];
    data[LF_THIGH] = rhs[LF_THIGH];
    data[LF_SHANK] = rhs[LF_SHANK];
    data[RF_HIP] = rhs[RF_HIP];
    data[RF_THIGH] = rhs[RF_THIGH];
    data[RF_SHANK] = rhs[RF_SHANK];
    data[LH_HIP] = rhs[LH_HIP];
    data[LH_THIGH] = rhs[LH_THIGH];
    data[LH_SHANK] = rhs[LH_SHANK];
    data[RH_HIP] = rhs[RH_HIP];
    data[RH_THIGH] = rhs[RH_THIGH];
    data[RH_SHANK] = rhs[RH_SHANK];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[BASE] = value;
    data[LF_HIP] = value;
    data[LF_THIGH] = value;
    data[LF_SHANK] = value;
    data[RF_HIP] = value;
    data[RF_THIGH] = value;
    data[RF_SHANK] = value;
    data[LH_HIP] = value;
    data[LH_THIGH] = value;
    data[LH_SHANK] = value;
    data[RH_HIP] = value;
    data[RH_THIGH] = value;
    data[RH_SHANK] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   base = "
    << map[BASE]
    << "   LF_HIP = "
    << map[LF_HIP]
    << "   LF_THIGH = "
    << map[LF_THIGH]
    << "   LF_SHANK = "
    << map[LF_SHANK]
    << "   RF_HIP = "
    << map[RF_HIP]
    << "   RF_THIGH = "
    << map[RF_THIGH]
    << "   RF_SHANK = "
    << map[RF_SHANK]
    << "   LH_HIP = "
    << map[LH_HIP]
    << "   LH_THIGH = "
    << map[LH_THIGH]
    << "   LH_SHANK = "
    << map[LH_SHANK]
    << "   RH_HIP = "
    << map[RH_HIP]
    << "   RH_THIGH = "
    << map[RH_THIGH]
    << "   RH_SHANK = "
    << map[RH_SHANK]
    ;
    return out;
}

}
}
#endif
