package org.codeorange.utility.drivers;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.codeorange.utility.Gains;
import org.codeorange.utility.OrangeUtility;
import org.codeorange.utility.TalonFXFactory;
import org.codeorange.utility.logging.MotorInputs;
import org.codeorange.utility.logging.TalonFXAutoLogger;

/**
 * SUPER CURSED!!! DO NOT USE THIS!!
 */
public abstract class ServoMotorSubsystemIOTalonFX implements ServoMotorSubsystemIO {

    public static class TalonFXConstants {
        public int CAN_ID = -1;
        public String CAN_BUS = "*";
        public InvertedValue invertedValue = InvertedValue.CounterClockwise_Positive;
    }

    public static class CANCoderConstants {
        public boolean exists = false;
        public int CAN_ID = -1;
        public String CAN_BUS = "*";
        public SensorDirectionValue invertedValue = SensorDirectionValue.CounterClockwise_Positive;
        public double encoderOffset;
    }

    public static class ServoMotorSubsystemConstants {
        public String name = "DEFAULT_NAME";

        public TalonFXConstants leaderConstants = new TalonFXConstants();
        public TalonFXConstants[] followerConstants = new TalonFXConstants[0];

        public Gains gains = new Gains();

        public double cruiseVelocity = 0;
        public double acceleration = 0;
        public double jerk = 9999999;

        public double supplyCurrentLimit = 60;
        public double statorCurrentLimit = 60;
        public boolean supplyCurrentLimitEnable = true;
        public boolean statorCurrentLimitEnable = false;

        public CANCoderConstants encoderConstants = new CANCoderConstants();
    }

    protected final ServoMotorSubsystemConstants constants;

    protected final TalonFX leader;
    protected final TalonFXConfiguration leaderConfig;
    protected final TalonFX[] followers;

    protected final TalonFXAutoLogger leaderLogger;
    protected final TalonFXAutoLogger[] followerLoggers;

    protected ServoMotorSubsystemIOTalonFX(ServoMotorSubsystemConstants newConstants) {
        constants = newConstants;
        leader = new TalonFX(constants.leaderConstants.CAN_ID, constants.leaderConstants.CAN_BUS);
        followers = new TalonFX[constants.followerConstants.length];
        followerLoggers = new TalonFXAutoLogger[constants.followerConstants.length];

        leaderLogger = new TalonFXAutoLogger(leader);

        leaderConfig = TalonFXFactory.getDefaultConfig();

        leaderConfig.Slot0.kP = constants.gains.kP();
        leaderConfig.Slot0.kI = constants.gains.kI();
        leaderConfig.Slot0.kD = constants.gains.kD();
        leaderConfig.Slot0.kS = constants.gains.kS();
        leaderConfig.Slot0.kV = constants.gains.kV();
        leaderConfig.Slot0.kA = constants.gains.kA();

        leaderConfig.MotionMagic.MotionMagicCruiseVelocity = constants.cruiseVelocity;
        leaderConfig.MotionMagic.MotionMagicAcceleration = constants.acceleration;
        leaderConfig.MotionMagic.MotionMagicJerk = constants.jerk;

        leaderConfig.CurrentLimits.SupplyCurrentLimit = constants.supplyCurrentLimit;
        leaderConfig.CurrentLimits.StatorCurrentLimit = constants.statorCurrentLimit;
        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = constants.supplyCurrentLimitEnable;
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = constants.statorCurrentLimitEnable;

        leaderConfig.MotorOutput.Inverted = constants.leaderConstants.invertedValue;

        if(constants.encoderConstants.exists) {
            var encoder = new CANcoder(constants.encoderConstants.CAN_ID, constants.encoderConstants.CAN_BUS);

            var configs = TalonFXFactory.getDefaultCANcoderConfig();

            configs.MagnetSensor.SensorDirection = constants.encoderConstants.invertedValue;
            configs.MagnetSensor.MagnetOffset = constants.encoderConstants.encoderOffset;

            OrangeUtility.betterCTREConfigApply(encoder, configs);

            leaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            leaderConfig.Feedback.FeedbackRemoteSensorID = constants.encoderConstants.CAN_ID;
        }

        OrangeUtility.betterCTREConfigApply(leader, leaderConfig);

        for (int i = 0; i < followers.length; i++) {
            followers[i] = TalonFXFactory.createFollower(constants.followerConstants[i].CAN_ID, constants.leaderConstants.CAN_ID);

            followers[i].setInverted((constants.followerConstants[i].invertedValue == InvertedValue.Clockwise_Positive));

            followerLoggers[i] = new TalonFXAutoLogger(followers[i]);
        }
    }

    protected final MotionMagicVoltage motionMagicReq = new MotionMagicVoltage(0).withSlot(0).withEnableFOC(true);

    @Override
    public void updateInputs(ServoMotorSubsystemInputs inputs) {
        if(inputs.followers.length != constants.followerConstants.length) {
            inputs.followers = new MotorInputs[constants.followerConstants.length];
        }
        inputs.leader = leaderLogger.log();
        for (int i = 0; i < followers.length; i++) {
            inputs.followers[i] = followerLoggers[i].log();
        }
    }

    @Override
    public void setPosition(double position) {
        leader.setControl(motionMagicReq.withPosition(position));
    }

    @Override
    public void zeroEncoder() {
        if(!constants.encoderConstants.exists) {
            leader.setPosition(0);
        }
    }

    protected final VoltageOut voltageReq = new VoltageOut(0).withEnableFOC(true);
    @Override
    public void setVoltage(double voltage) {
        leader.setControl(voltageReq.withOutput(voltage));
    }

    @Override
    public void stop() {
        leader.setControl(new NeutralOut());
    }
}
