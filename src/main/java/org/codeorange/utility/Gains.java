package org.codeorange.utility;

public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {
    public Gains() {
        this(0, 0, 0, 0, 0, 0);
    }
    public Gains(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0, 0);
    }
    public Gains(double kP, double kI, double kD, double kS) {
        this(kP, kI, kD, kS, 0, 0);
    }
}
