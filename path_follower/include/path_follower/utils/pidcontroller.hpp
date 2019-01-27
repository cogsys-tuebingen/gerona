#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <Eigen/Core>
#include <cslibs_navigation_utilities/Stopwatch.h>

/**
 * @brief Multidimensional PID controller.
 *
 * Implements a basic PID controller that is capable to work with multidimensional input (that
 * is it can actually controll multiple variables at onces).
 *
 * The first template parameter defines the number of variables. Parameters, errors, etc are
 * passed via Eigen::Vectors.
 * For the special case of dim=1, there are overloaded methods, that take floats/doubles.
 *
 * The second template Parameter can be used to change precision from float to double.
 *
 * Usage
 * -----
 *
 * One dimensional case:
 *
 *     PidController<1> pid_1d(1, 0.1, 0, 0.5); // P = 1, I = 0.1, D = 0, update interval = 0.5s
 *     float u;
 *     pid_1d.execute(error, &u);
 *
 * Multidimensional case:
 *
 *     typedef PidController<3>::Vector Vec3;
 *
 *     PidController<3> pid_3d(Vec3(1, 1,   0.5),
 *                             Vec3(0, 0.1, 0.2),
 *                             Vec3(0, 0,   0.1),
 *                             0.5);
 *     Vec3 u;
 *     pid_3d.execute(error, &u);
 *
 *     // --> u[0] is controlled with P = 1, I,D = 0
 *     //     u[1] is controlled with P = 1, I = 0.1, D = 0
 *     //     u[2] is controlled with P = 0.5, I = 0.2, D = 0.1
 */
template<unsigned int dim=1, typename real_t=float>
class PidController
{
public:
    typedef Eigen::Matrix<real_t, dim, 1> Vector;
    typedef Eigen::Matrix<real_t, dim, dim> Matrix;

    PidController():
        PidController(Vector::Zero(), Vector::Zero(), Vector::Zero(), 0)
    {}

    PidController(Vector K_p, Vector K_i, Vector K_d, real_t dt):
        K_p_(K_p.asDiagonal()),
        K_i_(K_i.asDiagonal()),
        K_d_(K_d.asDiagonal()),
        dt_(dt)
    {
        reset();
    }

    PidController(real_t K_p, real_t K_i, real_t K_d, real_t dt):
        PidController(Matrix(Matrix::Constant(K_p)),
                      Matrix(Matrix::Constant(K_i)),
                      Matrix(Matrix::Constant(K_d)),
                      dt)
    {
        // This constructor could also be disabled by setting the type of `K_p` to
        // `typename std::enable_if<dim == 1, real_t>::type`, but I found the error output of a
        // static_assert more helpful.
        static_assert(dim == 1,
                      "The scalar constuctor can only be used for one-dimensional PID"
                      " controllers");
    }

    //! Reset the controller.
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
        if (elapsed > 0.0 && elapsed >= dt_) {
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

    //! Overloaded method for the one-dimensional case
    bool execute(real_t error, real_t *u_out) {
        static_assert(dim == 1,
                      "The scalar execute() can only be used for one-dimensional PID"
                      " controllers");

        Vector vec_u;
        bool res = execute(Vector::Constant(error), &vec_u);
        *u_out = vec_u[0];
        return res;
    }

private:
    Matrix K_p_;
    Matrix K_i_;
    Matrix K_d_;
    real_t dt_;

    Vector previous_error_;
    Vector integral_;

    Stopwatch timer_;
};

#endif // PIDCONTROLLER_H
