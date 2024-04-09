package org.codeorange.utility.drivers;

import org.codeorange.utility.logging.MotorInputs;

/**
 * EXTREMELY CURSED, PROBABLY DOESN'T EVEN WORK
 */
public interface ServoMotorSubsystemIO extends SubsystemIO<ServoMotorSubsystemIO.ServoMotorSubsystemInputs> {
     class ServoMotorSubsystemInputs implements SubsystemIO.SubsystemInputs {
         public MotorInputs leader = new MotorInputs();
         public MotorInputs[] followers = new MotorInputs[0];
     }

     default void setPosition(double position) {}
     default void zeroEncoder() {}
     default void setVoltage(double voltage) {}
     default void stop() {}
}
