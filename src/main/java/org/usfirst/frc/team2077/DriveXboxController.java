package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj.*;

public class DriveXboxController extends XboxController implements DriveStick {
    protected double driveDeadBand, driveExponent;
    protected double rotationDeadBand, rotationExponent;

    public DriveXboxController(int port) {
        super(port);
    }

    public DriveXboxController setDriveSensitivity(double deadBand, double exponent) {
        this.driveDeadBand = deadBand;
        this.driveExponent = exponent;
        return this;
    }

    public DriveXboxController setRotationSensitivity(double deadBand, double exponent) {
        this.rotationDeadBand = deadBand;
        this.rotationExponent = exponent;
        return this;
    }

    public DriveXboxController setSensitivity(double deadBand, double exponent) {
        return setDriveSensitivity(deadBand, exponent).setRotationSensitivity(deadBand, exponent);
    }

    @Override public double getNorth() {
        return -DriveStick.adjustInputSensitivity(getLeftY(), driveDeadBand, driveExponent);
    }

    @Override public double getEast() {
        return -DriveStick.adjustInputSensitivity(getLeftX(), driveDeadBand, driveExponent);
    }

    @Override public double getRotation() {
        return DriveStick.adjustInputSensitivity(getRightX(), rotationDeadBand, rotationExponent);
    }
}
