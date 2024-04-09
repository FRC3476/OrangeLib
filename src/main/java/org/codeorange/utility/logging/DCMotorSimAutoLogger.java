package org.codeorange.utility.logging;

import org.codeorange.utility.simulation.BetterDCMotorSim;

public class DCMotorSimAutoLogger implements MotorAutoLogger {
    private final MotorInputs inputs;
    private final BetterDCMotorSim motorSim;

    public DCMotorSimAutoLogger(BetterDCMotorSim sim) {
        inputs = new MotorInputs();
        motorSim = sim;
    }

    public MotorInputs log() {
        inputs.CANID = -1;
        inputs.position = motorSim.getAngularPositionRotations();
        inputs.velocity = motorSim.getAngularVelocityRPM();
        inputs.supplyVoltage = 12.0;
        inputs.motorVoltage = motorSim.getInputVoltage();
        inputs.supplyCurrent = motorSim.getCurrentDrawAmps();
        inputs.statorCurrent = motorSim.getCurrentDrawAmps() / (inputs.motorVoltage / 12.5);
        inputs.temperature = 30;
        inputs.energy += inputs.supplyVoltage * inputs.supplyCurrent * 0.02;

        return inputs;
    }
}
