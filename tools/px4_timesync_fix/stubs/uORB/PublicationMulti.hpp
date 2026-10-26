#pragma once
#define ORB_ID(x) 0
namespace uORB {
template<typename T> struct PublicationMulti {
    explicit PublicationMulti(int) {}
    void publish(const T&) {}
};
}
