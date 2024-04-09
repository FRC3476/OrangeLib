package org.codeorange.utility.swerve;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import org.ejml.simple.SimpleMatrix;

import java.util.Arrays;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 * Helper class that extends default WPILib kinematics functionality to convert a chassis velocity
 * (dx, dy, and dtheta components) into individual module states (speed, angle, angular velocity).
 *
 * <p>The inverse kinematics (converting from a desired chassis velocity to individual module
 * states) uses the relative locations of the modules with respect to the center of rotation. The
 * center of rotation for inverse kinematics is also variable.
 *
 */
@SuppressWarnings("ReassignedVariable")
public class SecondOrderKinematics extends SwerveDriveKinematics {
    public static class SwerveDriveWheelStates extends SwerveDriveKinematics.SwerveDriveWheelStates {
        /** Swerve module states. */
        public SecondOrderModuleState[] states;

        /**
         * Creates a new SwerveDriveWheelStates instance.
         *
         * @param states The swerve module states. This will be deeply copied.
         */
        public SwerveDriveWheelStates(SecondOrderModuleState[] states) {
            super(states);
            this.states = new SecondOrderModuleState[states.length];
            for (int i = 0; i < states.length; i++) {
                this.states[i] = new SecondOrderModuleState(states[i].speedMetersPerSecond, states[i].angle, states[i].omega);
            }
        }
    }

    private final SimpleMatrix m_inverseKinematics;
    private final SimpleMatrix m_forwardKinematics;
    private final SimpleMatrix m_2ndOInvKinematics;

    private final int m_numModules;
    private final Translation2d[] m_modules;
    private Rotation2d[] m_moduleHeadings;
    private double[] m_omega;
    private Translation2d m_prevCoR = new Translation2d();

    /**
     * Constructs a second-order swerve drive kinematics object. This takes in a variable number of module
     * locations as Translation2d objects. The order in which you pass in the module locations is the
     * same order that you will receive the module states when performing inverse kinematics. It is
     * also expected that you pass in the module states in the same order when calling the forward
     * kinematics methods.
     *
     * @param moduleTranslationsMeters The locations of the modules relative to the physical center of
     *     the robot.
     */
    public SecondOrderKinematics(Translation2d... moduleTranslationsMeters) {
        super(moduleTranslationsMeters);
        if (moduleTranslationsMeters.length < 2) {
            throw new IllegalArgumentException("A swerve drive requires at least two modules");
        }
        m_numModules = moduleTranslationsMeters.length;
        m_modules = Arrays.copyOf(moduleTranslationsMeters, m_numModules);
        m_moduleHeadings = new Rotation2d[m_numModules];
        m_omega = new double[m_numModules];
        Arrays.fill(m_moduleHeadings, new Rotation2d());
        m_inverseKinematics = new SimpleMatrix(m_numModules * 2, 3);
        m_2ndOInvKinematics = new SimpleMatrix(m_numModules * 2, 4);

        for (int i = 0; i < m_numModules; i++) {
            m_inverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -m_modules[i].getY());
            m_inverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +m_modules[i].getX());
            m_2ndOInvKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -m_modules[i].getX(), -m_modules[i].getY());
            m_2ndOInvKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, -m_modules[i].getY(), +m_modules[i].getX());
        }
        m_forwardKinematics = m_inverseKinematics.pseudoInverse();

        MathSharedStore.reportUsage(MathUsageId.kKinematics_SwerveDrive, 1);
    }

    /**
     * Reset the internal swerve module headings.
     *
     * @param moduleHeadings The swerve module headings. The order of the module headings should be
     *     same as passed into the constructor of this class.
     */
    public void resetHeadings(Rotation2d... moduleHeadings) {
        if (moduleHeadings.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of headings is not consistent with number of module locations provided in "
                            + "constructor");
        }
        m_moduleHeadings = Arrays.copyOf(moduleHeadings, m_numModules);
    }

    /**
     * Performs inverse kinematics to return the module states from a desired chassis velocity. This
     * method is often used to convert joystick values into module speeds and angles.
     *
     * <p>This function also supports variable centers of rotation. During normal operations, the
     * center of rotation is usually the same as the physical center of the robot; therefore, the
     * argument is defaulted to that use case. However, if you wish to change the center of rotation
     * for evasive maneuvers, vision alignment, or for any other use case, you can do so.
     *
     * <p>In the case that the desired chassis speeds are zero (i.e. the robot will be stationary),
     * the previously calculated module angle will be maintained.
     *
     * @param chassisSpeeds The desired chassis speed.
     * @param centerOfRotationMeters The center of rotation. For example, if you set the center of
     *     rotation at one corner of the robot and provide a chassis speed that only has a dtheta
     *     component, the robot will rotate around that corner.
     * @return An array containing the module states. Use caution because these module states are not
     *     normalized. Sometimes, a user input may cause one of the module speeds to go above the
     *     attainable max velocity. Use the {@link #desaturateWheelSpeeds(SecondOrderModuleState[], double)
     *     DesaturateWheelSpeeds} function to rectify this issue.
     */
    public SecondOrderModuleState[] toSwerveModuleStates(
            ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
        var moduleStates = new SecondOrderModuleState[m_numModules];

        if (chassisSpeeds.vxMetersPerSecond == 0.0
                && chassisSpeeds.vyMetersPerSecond == 0.0
                && chassisSpeeds.omegaRadiansPerSecond == 0.0) {
            for (int i = 0; i < m_numModules; i++) {
                moduleStates[i] = new SecondOrderModuleState(0.0, m_moduleHeadings[i], m_omega[i]);
            }

            return moduleStates;
        }

        if (!centerOfRotationMeters.equals(m_prevCoR)) {
            for (int i = 0; i < m_numModules; i++) {
                m_inverseKinematics.setRow(
                        i * 2 + 0,
                        0, /* Start Data */
                        1,
                        0,
                        -m_modules[i].getY() + centerOfRotationMeters.getY());
                m_inverseKinematics.setRow(
                        i * 2 + 1,
                        0, /* Start Data */
                        0,
                        1,
                        +m_modules[i].getX() - centerOfRotationMeters.getX());
                m_2ndOInvKinematics.setRow(
                        i * 2 + 0,
                        0, /* Start Data */
                        1,
                        0,
                        -m_modules[i].getX() + centerOfRotationMeters.getX(),
                        -m_modules[i].getY() + centerOfRotationMeters.getY());
                m_2ndOInvKinematics.setRow(
                        i * 2 + 1,
                        0, /* Start Data */
                        0,
                        1,
                        -m_modules[i].getY() + centerOfRotationMeters.getY(),
                        +m_modules[i].getX() - centerOfRotationMeters.getX());
            }
            m_prevCoR = centerOfRotationMeters;
        }

        var chassisSpeedsVector = new SimpleMatrix(3, 1);
        chassisSpeedsVector.setColumn(
                0,
                0,
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond);

        var chassisAccelVector = new SimpleMatrix(4, 1);
        chassisAccelVector.setColumn(
                0,
                0,
                0,
                0,
                Math.pow(chassisSpeeds.omegaRadiansPerSecond, 2),
                0);

        var moduleStatesFirstOrder = m_inverseKinematics.mult(chassisSpeedsVector);
        var moduleStatesSecondOrder = m_2ndOInvKinematics.mult(chassisAccelVector);

        for (int i = 0; i < m_numModules; i++) {
            double vx = moduleStatesFirstOrder.get(i * 2 + 0, 0);
            double vy = moduleStatesFirstOrder.get(i * 2 + 1, 0);
            double ax = moduleStatesSecondOrder.get(i * 2 + 0, 0);
            double ay = moduleStatesSecondOrder.get(i * 2 + 1, 0);

            double speed = Math.hypot(vx, vy);
            Rotation2d angle = new Rotation2d(vx, vy);

            var thetaMatrix = new SimpleMatrix(2, 2);

            thetaMatrix.setRow(0, 0, Math.cos(angle.getRadians()), Math.sin(angle.getRadians()));
            thetaMatrix.setRow(1, 0, -Math.sin(angle.getRadians()), Math.cos(angle.getRadians()));

            var accelMatrix = new SimpleMatrix(2, 1);
            accelMatrix.setColumn(0, 0, ax, ay);

            var omega = (thetaMatrix.mult(accelMatrix).get(1,0) / speed) - chassisSpeeds.omegaRadiansPerSecond;

            moduleStates[i] = new SecondOrderModuleState(speed, angle, omega);
            m_moduleHeadings[i] = angle;
            m_omega[i] = omega;
        }

        return moduleStates;
    }

    /**
     * Performs inverse kinematics. See {@link #toSwerveModuleStates(ChassisSpeeds, Translation2d)}
     * toSwerveModuleStates for more information.
     *
     * @param chassisSpeeds The desired chassis speed.
     * @return An array containing the module states.
     */
    public SecondOrderModuleState[] toSwerveModuleStates(ChassisSpeeds chassisSpeeds) {
        return toSwerveModuleStates(chassisSpeeds, new Translation2d());
    }

    public SwerveDriveWheelStates toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
        return new SwerveDriveWheelStates(toSwerveModuleStates(chassisSpeeds));
    }

    /**
     * Performs forward kinematics to return the resulting chassis state from the given module states.
     * This method is often used for odometry -- determining the robot's position on the field using
     * data from the real-world speed and angle of each module on the robot.
     *
     * @param moduleStates The state of the modules (as a SwerveModuleState type) as measured from
     *     respective encoders and gyros. The order of the swerve module states should be same as
     *     passed into the constructor of this class.
     * @return The resulting chassis speed.
     */
    public ChassisSpeeds toChassisSpeeds(SecondOrderModuleState... moduleStates) {
        if (moduleStates.length != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of module locations provided in "
                            + "constructor");
        }
        var moduleStatesMatrix = new SimpleMatrix(m_numModules * 2, 1);

        for (int i = 0; i < m_numModules; i++) {
            var module = moduleStates[i];
            moduleStatesMatrix.set(i * 2, 0, module.speedMetersPerSecond * module.angle.getCos());
            moduleStatesMatrix.set(i * 2 + 1, module.speedMetersPerSecond * module.angle.getSin());
        }

        var chassisSpeedsVector = m_forwardKinematics.mult(moduleStatesMatrix);
        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0),
                chassisSpeedsVector.get(1, 0),
                chassisSpeedsVector.get(2, 0));
    }

    public ChassisSpeeds toChassisSpeeds(SwerveDriveWheelStates wheelStates) {
        return toChassisSpeeds(wheelStates.states);
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified maximum.
     *
     * <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
     * above the max attainable speed for the driving motor on that module. To fix this issue, one can
     * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
     * absolute threshold, while maintaining the ratio of speeds between modules.
     *
     * @param moduleStates Reference to array of module states. The array will be mutated with the
     *     normalized speeds!
     * @param attainableMaxSpeedMetersPerSecond The absolute max speed that a module can reach.
     */
    public static void desaturateWheelSpeeds(
            SecondOrderModuleState[] moduleStates, double attainableMaxSpeedMetersPerSecond) {
        double realMaxSpeed = 0;
        for (SecondOrderModuleState moduleState : moduleStates) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }
        if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
            for (SecondOrderModuleState moduleState : moduleStates) {
                moduleState.speedMetersPerSecond =
                        moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
            }
        }
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified maximum.
     *
     * <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
     * above the max attainable speed for the driving motor on that module. To fix this issue, one can
     * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
     * absolute threshold, while maintaining the ratio of speeds between modules.
     *
     * @param moduleStates Reference to array of module states. The array will be mutated with the
     *     normalized speeds!
     * @param attainableMaxSpeed The absolute max speed that a module can reach.
     */
    public static void desaturateWheelSpeeds(
            SecondOrderModuleState[] moduleStates, Measure<Velocity<Distance>> attainableMaxSpeed) {
        desaturateWheelSpeeds(moduleStates, attainableMaxSpeed.in(MetersPerSecond));
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified maximum, as well
     * as getting rid of joystick saturation at edges of joystick.
     *
     * <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
     * above the max attainable speed for the driving motor on that module. To fix this issue, one can
     * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
     * absolute threshold, while maintaining the ratio of speeds between modules.
     *
     * @param moduleStates Reference to array of module states. The array will be mutated with the
     *     normalized speeds!
     * @param desiredChassisSpeed The desired speed of the robot
     * @param attainableMaxModuleSpeedMetersPerSecond The absolute max speed that a module can reach
     * @param attainableMaxTranslationalSpeedMetersPerSecond The absolute max speed that your robot
     *     can reach while translating
     * @param attainableMaxRotationalVelocityRadiansPerSecond The absolute max speed the robot can
     *     reach while rotating
     */
    public static void desaturateWheelSpeeds(
            SecondOrderModuleState[] moduleStates,
            ChassisSpeeds desiredChassisSpeed,
            double attainableMaxModuleSpeedMetersPerSecond,
            double attainableMaxTranslationalSpeedMetersPerSecond,
            double attainableMaxRotationalVelocityRadiansPerSecond) {
        double realMaxSpeed = 0;
        for (SecondOrderModuleState moduleState : moduleStates) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }

        if (attainableMaxTranslationalSpeedMetersPerSecond == 0
                || attainableMaxRotationalVelocityRadiansPerSecond == 0
                || realMaxSpeed == 0) {
            return;
        }
        double translationalK =
                Math.hypot(desiredChassisSpeed.vxMetersPerSecond, desiredChassisSpeed.vyMetersPerSecond)
                        / attainableMaxTranslationalSpeedMetersPerSecond;
        double rotationalK =
                Math.abs(desiredChassisSpeed.omegaRadiansPerSecond)
                        / attainableMaxRotationalVelocityRadiansPerSecond;
        double k = Math.max(translationalK, rotationalK);
        double scale = Math.min(k * attainableMaxModuleSpeedMetersPerSecond / realMaxSpeed, 1);
        for (SecondOrderModuleState moduleState : moduleStates) {
            moduleState.speedMetersPerSecond *= scale;
        }
    }

    /**
     * Renormalizes the wheel speeds if any individual speed is above the specified maximum, as well
     * as getting rid of joystick saturation at edges of joystick.
     *
     * <p>Sometimes, after inverse kinematics, the requested speed from one or more modules may be
     * above the max attainable speed for the driving motor on that module. To fix this issue, one can
     * reduce all the wheel speeds to make sure that all requested module speeds are at-or-below the
     * absolute threshold, while maintaining the ratio of speeds between modules.
     *
     * @param moduleStates Reference to array of module states. The array will be mutated with the
     *     normalized speeds!
     * @param desiredChassisSpeed The desired speed of the robot
     * @param attainableMaxModuleSpeed The absolute max speed that a module can reach
     * @param attainableMaxTranslationalSpeed The absolute max speed that your robot can reach while
     *     translating
     * @param attainableMaxRotationalVelocity The absolute max speed the robot can reach while
     *     rotating
     */
    public static void desaturateWheelSpeeds(
            SecondOrderModuleState[] moduleStates,
            ChassisSpeeds desiredChassisSpeed,
            Measure<Velocity<Distance>> attainableMaxModuleSpeed,
            Measure<Velocity<Distance>> attainableMaxTranslationalSpeed,
            Measure<Velocity<Angle>> attainableMaxRotationalVelocity) {
        desaturateWheelSpeeds(
                moduleStates,
                desiredChassisSpeed,
                attainableMaxModuleSpeed.in(MetersPerSecond),
                attainableMaxTranslationalSpeed.in(MetersPerSecond),
                attainableMaxRotationalVelocity.in(RadiansPerSecond));
    }
}
