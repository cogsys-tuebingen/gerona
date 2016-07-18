#include "transition.h"

double Transition::arc_length() const
{
    return std::abs(dtheta * r);
}
