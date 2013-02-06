/*
 * Intermission.hpp
 *
 *  Created on: Feb 06, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef INTERMISSION_HPP
#define INTERMISSION_HPP

namespace generic {

template <int steps>
struct CallbackIntermission {
    template <class Callback, class A>
    static void callAndCopy(Callback& c, const A& from, A& to) {
        static int next = steps;
        if(--next <= 0) {
            to = from;
            c();
            next = steps;
        }
    }
    template <class Callback>
    static void call(Callback& c) {
        static int next = steps;
        if(--next <= 0) {
            c();
            next = steps;
        }
    }
};
template<>
struct CallbackIntermission<0> {
    template <class Callback, class A>
    static void callAndCopy(Callback& c, const A& from, A& to) {}
    template <class Callback>
    static void call(Callback& c) {}
};

struct NoIntermission {
    template <class Ignore, class A>
    static void callAndCopy(Ignore& i, const A& from, A& to) {}
    template <class Ignore>
    static void call(Ignore& i) {}
};

}

#endif // INTERMISSION_HPP
