#include "multiplepidwrapper.h"

using namespace std;

MultiplePidWrapper::MultiplePidWrapper(size_t n):
    controllers_(n)
{
}

void MultiplePidWrapper::resetAll()
{
    vector<PidCtrl>::iterator it;
    for (it = controllers_.begin(); it != controllers_.end(); ++it) {
        it->reset();
    }
}

void MultiplePidWrapper::configure(size_t idx, double Kp, double delta_max, double e_max, double v, double Ki, double i_max)
{
    // ta = 0, as there is a external stopwatch for all controllers.
    controllers_[idx].configure(Kp, Ki, i_max, delta_max, e_max, v, 0.0);
}

bool MultiplePidWrapper::execute(double errors[], std::vector<double>& result)
{
    double d_t = timer_.msElapsed();

    if (d_t >= Ta_*1000.0) {
        result = vector<double>(controllers_.size());
        for (size_t i = 0; i < controllers_.size(); ++i) {
            controllers_[i].execute(errors[i], result[i]);
        }

        return true;
    } else {
        return false;
    }
}
