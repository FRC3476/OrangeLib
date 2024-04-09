package org.codeorange.utility.logging;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;

/**
 * Logs desired signals from a given TalonFX motor controller.
 */
public class TalonFXAutoLogger implements MotorAutoLogger {
    private final MotorInputs inputs;

    private final StatusSignal<Double> position;
    private final StatusSignal<Double> velocity;
    private final StatusSignal<Double> supplyCurrent;
    private final StatusSignal<Double> statorCurrent;
    private final StatusSignal<Double> supplyVoltage;
    private final StatusSignal<Double> motorVoltage;
    private final StatusSignal<Double> temperature;

    public TalonFXAutoLogger(TalonFX motor) {
        inputs = new MotorInputs();
        inputs.CANID = motor.getDeviceID();
        position = motor.getPosition();
        velocity = motor.getVelocity();
        supplyCurrent = motor.getSupplyCurrent();
        statorCurrent = motor.getStatorCurrent();
        supplyVoltage = motor.getSupplyVoltage();
        motorVoltage = motor.getMotorVoltage();
        temperature = motor.getDeviceTemp();
    }
    public MotorInputs log() {
        BaseStatusSignal.refreshAll(
            position,
            velocity,
            supplyCurrent,
            statorCurrent,
            supplyVoltage,
            motorVoltage,
            temperature
        );

        inputs.position = position.getValue();
        inputs.velocity = velocity.getValue();
        inputs.supplyCurrent = supplyCurrent.getValue();
        inputs.statorCurrent = statorCurrent.getValue();
        inputs.supplyVoltage = supplyVoltage.getValue();
        inputs.motorVoltage = motorVoltage.getValue();
        inputs.temperature = temperature.getValue();
        inputs.energy += inputs.supplyVoltage * inputs.supplyCurrent * 0.02;

        return inputs;
    }
}
