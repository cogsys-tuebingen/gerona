#ifndef MOVECOMMAND_H
#define MOVECOMMAND_H

#include <Eigen/Core>

/**
 * @brief Model independent move command.
 *
 * This class represents a basic move command that should be independent of the robot model and
 * controller type. It is used by the obstacle avoider to make them as universal as possible.
 * Conversion to and from the model specific commands has to be done by the particular
 * controller
 *
 * Note that not all robot models support a direct rotation (e.g. car-like model does not),
 * therefore the rotation value is meaningless for this robot types and should not be used, for
 * example by the obstacle avoider. To ensure this, rotation support has to be enabled
 * explicitly by setting the constructor argument `can_rotate` to true.
 */
class MoveCommand
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Constructor.
     * @param can_rotate Set this to true if the used robot model supports direct rotation
     *                   commands.
     */
    MoveCommand(bool can_rotate = false);

    //! Check if the command is valid (= there are no nan or inf values).
    bool isValid() const;

    //! Unit vector pointing in the direction of movement.
    Eigen::Vector2f getDirection() const;
    //! Vector pointing in the direction of movement, scaled by the velocity.
    Eigen::Vector2f getVelocityVector() const;
    //! Angle to the direction of movement.
    float getDirectionAngle() const;
    //! Scalar velocity.
    float getVelocity() const;

    //! True, iff the robot supports rotation commands.
    bool hasRotation() const;
    //! Desired rotation as angle. Undefined if hasOrientation() == false.
    float getRotation() const;

    //! Set direction vector (length of the vector is ignored)
    void setDirection(const Eigen::Vector2f &dir);
    //! Set direction represented by an angle.
    void setDirection(float angle);
    //! Set scalar velocity.
    void setVelocity(float v);
    //! Set rotation angle.
    void setRotation(float o); //TODO: better name?

private:
    //! Unit vector pointing in the direction of movement.
    Eigen::Vector2f move_dir_;
    //! Scalar velocity
    float velocity_;

    //! Angle of desired rotation.
    float rotation_;
    //! If false, rotation_ is undefined and must not be used.
    bool use_rotation_;

    //! Check if the given value is neither NaN nor +/-infinity.
    bool isValid(float val) const;
};

#endif // MOVECOMMAND_H
