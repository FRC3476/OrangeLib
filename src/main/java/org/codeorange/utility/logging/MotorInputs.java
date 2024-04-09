package org.codeorange.utility.logging;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

import java.nio.ByteBuffer;

/**
 * Represents various desired fields from a motor controller.
 */
public class MotorInputs implements StructSerializable {
    public double CANID;
    public double position;
    public double velocity;
    public double supplyCurrent;
    public double statorCurrent;
    public double supplyVoltage;
    public double motorVoltage;
    public double temperature;
    public double energy;

    public MotorInputs(double CANID, double position, double velocity, double supplyCurrent, double statorCurrent, double supplyVoltage, double motorVoltage, double temperature, double energy) {
        this.CANID = CANID;
        this.position = position;
        this.velocity = velocity;
        this.supplyCurrent = supplyCurrent;
        this.statorCurrent = statorCurrent;
        this.supplyVoltage = supplyVoltage;
        this.motorVoltage = motorVoltage;
        this.temperature = temperature;
        this.energy = energy;
    }

    public MotorInputs() {
        this(0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public static final MotorInputsStruct struct = new MotorInputsStruct();

    public static class MotorInputsStruct implements Struct<MotorInputs> {
        @Override
        public Class<MotorInputs> getTypeClass() {
            return MotorInputs.class;
        }

        @Override
        public String getTypeString() {
            return "struct:MotorInputs";
        }

        @Override
        public int getSize() {
            return kSizeDouble * 9;
        }

        @Override
        public String getSchema() {
            return "double CANID;double position;double velocity;double supplyCurrent;double statorCurrent;double supplyVoltage;double motorVoltage;double temperature;double energy;";
        }

        @Override
        public MotorInputs unpack(ByteBuffer bb) {
            double CANID = bb.getDouble();
            double position = bb.getDouble();
            double velocity = bb.getDouble();
            double supplyCurrent = bb.getDouble();
            double statorCurrent = bb.getDouble();
            double supplyVoltage = bb.getDouble();
            double motorVoltage = bb.getDouble();
            double temperature = bb.getDouble();
            double energy = bb.getDouble();

            return new MotorInputs(CANID, position, velocity, supplyCurrent, statorCurrent, supplyVoltage, motorVoltage, temperature, energy);
        }

        @Override
        public void pack(ByteBuffer bb, MotorInputs value) {
            bb.putDouble(value.CANID);
            bb.putDouble(value.position);
            bb.putDouble(value.velocity);
            bb.putDouble(value.supplyCurrent);
            bb.putDouble(value.statorCurrent);
            bb.putDouble(value.supplyVoltage);
            bb.putDouble(value.motorVoltage);
            bb.putDouble(value.temperature);
            bb.putDouble(value.energy);
        }
    }
}
