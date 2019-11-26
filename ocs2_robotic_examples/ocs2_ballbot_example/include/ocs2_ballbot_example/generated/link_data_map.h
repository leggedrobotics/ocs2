#ifndef IIT_BALLBOT_LINK_DATA_MAP_H_
#define IIT_BALLBOT_LINK_DATA_MAP_H_

#include "declarations.h"

namespace iit {
namespace Ballbot {

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
    data[WORLD] = rhs[WORLD];
    data[DUMMY_BALL1] = rhs[DUMMY_BALL1];
    data[BALL] = rhs[BALL];
    data[DUMMY_BASE1] = rhs[DUMMY_BASE1];
    data[DUMMY_BASE2] = rhs[DUMMY_BASE2];
    data[BASE] = rhs[BASE];
}

template<typename T> inline
void LinkDataMap<T>::assigndata(const T& value) {
    data[WORLD] = value;
    data[DUMMY_BALL1] = value;
    data[BALL] = value;
    data[DUMMY_BASE1] = value;
    data[DUMMY_BASE2] = value;
    data[BASE] = value;
}

template<typename T> inline
std::ostream& operator<<(std::ostream& out, const LinkDataMap<T>& map) {
    out
    << "   world = "
    << map[WORLD]
    << "   dummy_ball1 = "
    << map[DUMMY_BALL1]
    << "   ball = "
    << map[BALL]
    << "   dummy_base1 = "
    << map[DUMMY_BASE1]
    << "   dummy_base2 = "
    << map[DUMMY_BASE2]
    << "   base = "
    << map[BASE]
    ;
    return out;
}

}
}
#endif
