package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj.*;

public class DriveJoystick extends Joystick implements DriveStick {
    private static final int DEFAULT_ROTATION_AXIS = AxisType.kZ.value;

    protected double driveDeadBand, driveExponent;
    protected double rotationDeadBand, rotationExponent;
    protected int rotationAxis;

    /**
     * Construct an instance of a joystick.
     *
     * @param port The port index on the Driver Station that the joystick is plugged into.
     */
    public DriveJoystick(int port, int rotationAxis) {
        super(port);
        this.rotationAxis = rotationAxis;
    }

    public DriveJoystick(int port) {
        this(port, DEFAULT_ROTATION_AXIS);
    }

    public DriveJoystick setDriveSensitivity(double deadBand, double exponent) {
        this.driveDeadBand = deadBand;
        this.driveExponent = exponent;
        return this;
    }

    public DriveJoystick setRotationSensitivity(double deadBand, double exponent) {
        this.rotationDeadBand = deadBand;
        this.rotationExponent = exponent;
        return this;
    }

    public DriveJoystick setSensitivity(double deadBand, double exponent) {
        return setDriveSensitivity(deadBand, exponent).setRotationSensitivity(deadBand, exponent);
    }

    public double getNorth() {
        return DriveStick.adjustInputSensitivity(getY(), driveDeadBand, driveExponent);
    }

    public double getEast() {
        return DriveStick.adjustInputSensitivity(getX(), driveDeadBand, driveExponent);
    }

    public double getRotation() {
        return DriveStick.adjustInputSensitivity(getRawAxis(rotationAxis), rotationDeadBand, rotationExponent);
    }

}
