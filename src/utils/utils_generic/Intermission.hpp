/*
 * Intermission.hpp
 *
 *  Created on: Feb 06, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef INTERMISSION_HPP
#define INTERMISSION_HPP

namespace generic
{

template <int steps, int start_step = 0>
struct CallbackIntermission {
    template <class Callback, class A>
    static void callAndCopy(Callback& c, const A& from, A& to) {
        static int next = start_step;
        static bool start = true;
        if(--next <= 0 || start) {
            to = from;
            c();
            if(next > 0) return;
            next = steps;
            start = false;
        }
    }
    template <class Callback>
    static void call(Callback& c) {
        static int next = start_step;
        static bool start = true;
        if(--next <= 0 || start) {
            c();
            if(next > 0) return;
            next = steps;
            start = false;
        }
    }
};
template<int start_step>
struct CallbackIntermission<0, start_step> {
    template <class Callback, class A>
    static void callAndCopy(Callback& c, const A& from, A& to) {}
    template <class Callback>
    static void call(Callback& c) {}
};

struct NoIntermission {
    template <class Ignore, class A>
    static void callAndCopy(Ignore& /*i*/, const A& /*from*/, A& /*to*/) {}
    template <class Ignore>
    static void call(Ignore& i) {}
};

}

#endif // INTERMISSION_HPP
