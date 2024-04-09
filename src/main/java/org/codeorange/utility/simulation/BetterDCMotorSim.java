package org.codeorange.utility.simulation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.codeorange.utility.wpimodified.PIDController;

/**
 * DCMotorSim but stores input voltage and has built in gain calculation because i am lazy
 */
public class BetterDCMotorSim extends DCMotorSim {
    private double inputVoltage = 0;
    private PIDController pidController = new PIDController(0, 0, 0);
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    public void setPID(PIDController controller) {
        pidController = controller;
    }

    public void setFeedforward(SimpleMotorFeedforward ff) {
        feedforward = ff;
    }

    public void setGains(PIDController controller, SimpleMotorFeedforward ff) {
        setPID(controller);
        setFeedforward(ff);
    }

    public BetterDCMotorSim(LinearSystem<N2, N1, N2> plant, DCMotor gearbox, double gearing) {
        super(plant, gearbox, gearing);
    }

    public BetterDCMotorSim(LinearSystem<N2, N1, N2> plant, DCMotor gearbox, double gearing, Matrix<N2, N1> measurementStdDevs) {
        super(plant, gearbox, gearing, measurementStdDevs);
    }

    public BetterDCMotorSim(DCMotor gearbox, double gearing, double jKgMetersSquared) {
        super(gearbox, gearing, jKgMetersSquared);
    }

    public BetterDCMotorSim(DCMotor gearbox, double gearing, double jKgMetersSquared, Matrix<N2, N1> measurementStdDevs) {
        super(gearbox, gearing, jKgMetersSquared, measurementStdDevs);
    }

    public BetterDCMotorSim(DCMotor gearbox, double gearing, double jKgMetersSquared, PIDController pidController, SimpleMotorFeedforward feedforward) {
        this(gearbox, gearing, jKgMetersSquared);
        this.pidController = pidController;
        this.feedforward = feedforward;
    }

    @Override
    public void setInputVoltage(double volts) {
        inputVoltage = MathUtil.clamp(volts, -12, 12);
        setInput(inputVoltage);
    }

    public double getInputVoltage() {
        return inputVoltage;
    }

    public void setPosition(double setpoint) {
        setPosition(setpoint, 0);
    }

    public void setPosition(double setpoint, double ffVolts) {
        setInputVoltage(
                pidController.calculate(getAngularPositionRotations(), setpoint)
                + ffVolts
        );
    }

    public void setVelocity(double setpoint) {
        setInputVoltage(
                feedforward.calculate(setpoint)
                + pidController.calculate(getAngularVelocityRPS(), setpoint)
        );
    }

    public double getAngularVelocityRPS() {
        return Units.radiansToRotations(getAngularVelocityRadPerSec());
    }
}
