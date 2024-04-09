package org.codeorange.utility.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Represents the state of one swerve module, including angular velocity. */
public class SecondOrderModuleState extends SwerveModuleState {
    /** Speed of the wheel of the module. */
    public double speedMetersPerSecond;

    /** Angle of the module. */
    public Rotation2d angle;

    /** Angular velocity of the module. */
    public double omega;

    /**
     * Constructs a SecondOrderModuleState.
     *
     * @param speedMetersPerSecond The speed of the wheel of the module.
     * @param angle The angle of the module.
     * @param omega The angular velocity of the module.
     */
    public SecondOrderModuleState(double speedMetersPerSecond, Rotation2d angle, double omega) {
        this.speedMetersPerSecond = speedMetersPerSecond;
        this.angle = angle;
        this.omega = omega;
    }

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Angular velocity is unchanged.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     * @return Optimized swerve module state.
     */
    public static SecondOrderModuleState optimize(
            SecondOrderModuleState desiredState, Rotation2d currentAngle) {
        var delta = desiredState.angle.minus(currentAngle);
        if (delta.getCos() < 0) {
            return new SecondOrderModuleState(
                    -desiredState.speedMetersPerSecond,
                    desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)),
                    desiredState.omega);
        } else {
            return new SecondOrderModuleState(desiredState.speedMetersPerSecond, desiredState.angle, desiredState.omega);
        }
    }

    /**
     * Creates a first-order module state equivalent by removing the omega value. This is useful for
     * features that are only built to support WPILib's default SwerveModuleState class, such as
     * AdvantageKit's swerve graph.
     *
     * @return First-order module state.
     */
    public SwerveModuleState toFirstOrder() {
        return new SwerveModuleState(speedMetersPerSecond, angle);
    }
}
