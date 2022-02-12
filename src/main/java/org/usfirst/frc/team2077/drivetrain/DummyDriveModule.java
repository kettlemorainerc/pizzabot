/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.drivetrain;

public class DummyDriveModule implements DriveModule {

    // velocity set point
    private double velocity_ = 0;

    // accumulated distance since last velocity change or distance reset
    private double distance_ = 0;

    // time of last distance reset
    private long distanceTime_ = System.currentTimeMillis();

    @Override
    public void setVelocity(double velocity) {
        long now = System.currentTimeMillis();
        distance_ += velocity_ * ((now-distanceTime_)/1000.);
        distanceTime_ = now;
        velocity_ = velocity;
    }

    @Override
    public double getVelocity() {
        return velocity_;
    }

    @Override
    public double getDistance() {
        long now = System.currentTimeMillis();
        return distance_ + velocity_ * ((now-distanceTime_)/1000.);
    }

    @Override
    public void resetDistance() {
        distance_ = 0;
        distanceTime_ = System.currentTimeMillis();
    
    }
}