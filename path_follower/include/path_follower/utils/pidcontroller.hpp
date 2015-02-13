#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <type_traits> // for std::conditional
#include <Eigen/Core>
#include <utils_general/Stopwatch.h>

/**
 * @brief Multidimensional PID controller
 */
template<unsigned int dim>
class PidController
{
public:
    // typedef for vector and matrix type enables easy switch to double precision if needed.
    typedef float real_t;

    // use C++11 magic to make Vector and Matrix simple scalars, if dim == 1 and use Eigen
    // if dim > 1
    typedef typename std::conditional<dim == 1,
                                      real_t,
                                      Eigen::Matrix<real_t, dim, 1> >::type Vector;
    typedef typename std::conditional<dim == 1,
                                      real_t,
                                      Eigen::Matrix<real_t, dim, dim> >::type Matrix;

    PidController():
        PidController(Vector::Zero(), Vector::Zero(), Vector::Zero(), 0)
    {}

    PidController(Vector K_p, Vector K_i, Vector K_d, real_t dt):
        K_p_(Matrix::Identity() * K_p), //FIXME: this is not working with the conditional type
        K_i_(Matrix::Identity() * K_i),
        K_d_(Matrix::Identity() * K_d),
        dt_(dt)
    {
        reset();
    }

    void reset()
    {
        previous_error_ = Vector::Zero();
        integral_ = Vector::Zero();
        timer_.restart();
    }


    /**
     * @brief Execute one control cycle if enough time has elapsed since last call.
     * @param error The error between desired and real value (`error = x_desired - x_real`)
     * @param u_out Output. The control command. Only set if method returns true!
     * @return True if enougth time has elapsed and u_out is set. False if not.
     */
    bool execute(const Vector &error, Vector *u_out)
    {
        /*
         * Pseudocode from Wikipedia:
         *
         * previous_error = 0
         * integral = 0
         * start:
         *   error = setpoint - measured_value
         *   integral = integral + error*dt
         *   derivative = (error - previous_error)/dt
         *   output = Kp*error + Ki*integral + Kd*derivative
         *   previous_error = error
         *   wait(dt)
         *   goto start
         *
         */

        // Only compute new u, if at least dt_ time has elapsed.
        double elapsed = timer_.msElapsed() / 1000.0;
        if (elapsed >= dt_) {
            // Use the exact elapsed time (`elapsed`) instead of `dt_`
            integral_ += error * elapsed;
            Vector derivative = (error - previous_error_) / elapsed;
            *u_out = K_p_ * error + K_i_ * integral_ + K_d_ * derivative;
            previous_error_ = error;

            timer_.restart();
            return true;
        } else {
            return false;
        }
    }

private:
    const Matrix K_p_;
    const Matrix K_i_;
    const Matrix K_d_;
    const real_t dt_;

    Vector previous_error_;
    Vector integral_;

    Stopwatch timer_;
};

#endif // PIDCONTROLLER_H
