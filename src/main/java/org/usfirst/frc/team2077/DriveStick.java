package org.usfirst.frc.team2077;

public interface DriveStick {
    static double adjustInputSensitivity(double input, double deadBand, double exponent) {
        return Math.pow(Math.max(0, Math.abs(input) - deadBand) / (1 - deadBand), exponent) * Math.signum(input);
    }

    public double getNorth();
    public double getEast();
    public double getRotation();
}
