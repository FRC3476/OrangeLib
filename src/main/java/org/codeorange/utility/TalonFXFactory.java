package org.codeorange.utility;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonFXFactory {
    private static final NeutralModeValue neutralMode = NeutralModeValue.Coast;
    private static final InvertedValue inverted = InvertedValue.CounterClockwise_Positive;

    public static TalonFXConfiguration getDefaultConfig() {
        var config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = neutralMode;
        config.MotorOutput.Inverted = inverted;
        config.MotorOutput.PeakForwardDutyCycle = 1;
        config.MotorOutput.PeakReverseDutyCycle = -1;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 120;
        config.CurrentLimits.StatorCurrentLimitEnable = false;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = 1;

        config.Audio.BeepOnBoot = true;
        config.Audio.BeepOnConfig = true;

        return config;
    }

    public static TalonFX createDefault(int id) {
        return createDefault(id, "*");
    }

    public static TalonFX createDefault(int id, String CANBUS) {
        TalonFX motor = new TalonFX(id, CANBUS);
        OrangeUtility.betterCTREConfigApply(motor, getDefaultConfig());
        motor.clearStickyFaults();

        return motor;
    }
}
