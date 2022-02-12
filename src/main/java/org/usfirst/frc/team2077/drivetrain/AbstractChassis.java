package org.usfirst.frc.team2077.drivetrain;

import static org.usfirst.frc.team2077.Robot.*;

public abstract class AbstractChassis implements DriveChassis {

    protected final DriveModule[] driveModule_;
    protected final double wheelbase_;
    protected final double trackWidth_;
    protected final double wheelRadius_;

    public AbstractChassis(DriveModule[] driveModule, double wheelbase, double trackWidth, double wheelRadius) {
        driveModule_ = driveModule;
        wheelbase_ = wheelbase;
        trackWidth_ = trackWidth;
        wheelRadius_ = wheelRadius;
    }

    @Override
    public void moveAbsolute(double north, double east, double heading) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void moveAbsolute(double north, double east) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void rotateAbsolute(double clockwise) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void moveRelative(double north, double east, double heading) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void moveRelative(double north, double east) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void rotateRelative(double clockwise) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setPosition() {
        throw new UnsupportedOperationException();
    }

    @Override
    public double[] getPosition() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setVelocity(double north, double east, double clockwise) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setVelocity(double north, double east) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setRotation(double clockwise) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setVelocity01(double north, double east, double clockwise) {
        setVelocity(north*robot_.constants_.TOP_SPEED, east*robot_.constants_.TOP_SPEED, clockwise); // TODO: Scaling rule for rotation.
    }

    @Override
    public void setVelocity01(double north, double east) {
        setVelocity(north*robot_.constants_.TOP_SPEED, east*robot_.constants_.TOP_SPEED);
    }

    @Override
    public void setRotation01(double clockwise) {
        setRotation(clockwise*robot_.constants_.TOP_SPEED);
    }

    @Override
    public void stop() {
        setVelocity(0, 0, 0);
    }

    @Override
    public abstract double[] getVelocitySet();

    @Override
    public double[] getVelocityCalculated() {
        throw new UnsupportedOperationException();
    }

    @Override
    public double[] getVelocityMeasured() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setAccelerationLimits(double accelerationLimit, double decelerationLimit) {
    }

    @Override
    public double[] getAccelerationLimits() {
        return new double[] {Double.MAX_VALUE, Double.MAX_VALUE};
    }

}