/*
 * DynamicPriorityQueue.hpp
 *
 *  Created on: May 6, 2013
 *      Author: buck <sebastian.buck@uni-tuebingen.de>
 */

#ifndef DYNAMICPRIORITYQUEUE_H
#define DYNAMICPRIORITYQUEUE_H

/// SYSTEM
#include <queue>
#include <iostream>
#include <set>
#include <assert.h>

template<typename _Key, typename _Compare  = std::less<_Key> >
class DynamicPrioritySet : public std::multiset<_Key, _Compare>
{
    typedef std::multiset<_Key, _Compare> S;

    typedef _Key     key_type;
    typedef _Key     value_type;
    typedef _Compare key_compare;
    typedef _Compare value_compare;
    typedef typename S::size_type size_type;
    typedef typename S::const_reference const_reference;
    typedef typename S::iterator iterator;
    typedef typename S::const_iterator const_iterator;

public:

    bool empty() const
    {
        return S::empty();
    }

    size_type size() const
    {
        return S::size();
    }

    void validate() const
    {
//        key_compare comp = S::key_comp();
//        for(const_iterator i = S::begin(); i != S::end(); ++i) {
//            const_iterator next = i;
//            next++;

//            if(next == S::end()) {
//                return;
//            }
//            assert(comp(*i, *next));
//        }
    }

    value_type top() const
    {

        validate();
        return *--S::end();
    }

    void push(const value_type& __x)
    {
        S::insert(__x);

        validate();
    }

    void pop()
    {
        S::erase(--S::end());
    }

    void remove(const value_type& __x)
    {
//        typename S::iterator pos = S::find(__x);
//        if(pos != S::end()) {
//            S::erase(pos);
//        } else {
//            std::cout << "cannot remove: " << __x << " not found" << std::endl;
            int c = 0;
            for(const_iterator i = S::begin(); i != S::end(); ++i) {
                if(*i == __x) {
                    S::erase(i);
                    return;
                }
                ++c;
            }
//            std::cout << std::endl;
//        }
    }
};

template<typename _Tp, typename _Sequence = std::vector<_Tp>,
         typename _Compare  = std::less<typename _Sequence::value_type> >
class DynamicPriorityQueue : public std::priority_queue<_Tp, _Sequence, _Compare>
{
    typedef std::priority_queue<_Tp, _Sequence, _Compare> PQ;

public:
    typedef typename _Sequence::value_type                value_type;
    typedef typename _Sequence::reference                 reference;
    typedef typename _Sequence::const_reference           const_reference;
    typedef typename _Sequence::size_type                 size_type;
    typedef          _Sequence                            container_type;

    using PQ::c;
    using PQ::comp;

public:
    bool empty() const
    {
        return PQ::empty();
    }

    size_type size() const
    {
        return PQ::size();
    }

    const_reference top() const
    {
        return PQ::top();
    }

    void push(const value_type& __x)
    {
        c.push_back(__x);
        std::push_heap(c.begin(), c.end(), comp);
    }

    void pop()
    {
        std::pop_heap(c.begin(), c.end(), comp);
        c.pop_back();
    }

    void remove(const value_type& __x)
    {
        //std::make_heap(c.begin(), c.end(), comp);
        Extractor<_Compare> extractor(__x, comp);
        std::pop_heap(c.begin(), c.end(), extractor);
        c.pop_back();
    }

private:
    template <class Comp>
    struct Extractor {
        Extractor(const value_type& __x, const Comp& c)
            : elem(__x), comp(c)
        {}

        bool operator() (const value_type& lhs, const value_type& rhs) const
        {
            if(lhs == elem) {
                return true;
            } else {
                return comp(lhs, rhs);
            }
        }

        const value_type& elem;
        const Comp& comp;
    };
};

#endif // DYNAMICPRIORITYQUEUE_H
