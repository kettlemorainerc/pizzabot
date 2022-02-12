/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.drivetrain;

public interface DriveChassis {

    void moveAbsolute(double north, double east, double heading);
    void moveAbsolute(double north, double east);
    void rotateAbsolute(double clockwise);
    
    void moveRelative(double north, double east, double heading);
    void moveRelative(double north, double east);
    void rotateRelative(double clockwise);

    void setPosition();

    double[] getPosition();

    /**
     * 
     * @param north In distance units per second.
     * @param east In distance units per second.
     * @param clockwise In degrees per second.
     */
    void setVelocity(double north, double east, double clockwise);
    /**
     * 
     * @param north In distance units per second.
     * @param east In distance units per second.
     */
    void setVelocity(double north, double east);
    /**
     * 
     * @param clockwise In degrees per second.
     */
    void setRotation(double clockwise);

    /**
     * 
     * @param north Fraction of nominal maximum in range -1.0 to 1.0.
     * @param east Fraction of nominal maximum in range -1.0 to 1.0.
     * @param clockwise Fraction of nominal maximum in range -1.0 to 1.0.
     */
    void setVelocity01(double north, double east, double clockwise);
    /**
     * 
     * @param north Fraction of nominal maximum in range -1.0 to 1.0.
     * @param east Fraction of nominal maximum in range -1.0 to 1.0.
     */
    void setVelocity01(double north, double east);
    /**
     * 
     * @param clockwise Fraction of nominal maximum in range -1.0 to 1.0.
     */
    void setRotation01(double clockwise);

    /**
     * Set all drives to zero immediately, ignoring any deceleration limits.
     */
    void stop();

    /**
     * Velocity set point.
     * @return {north, east, rotation} Units are distance units or degrees per second.
     */
    double[] getVelocitySet();

    /**
     * Internal velocity set point.
     * May be affected by acceleration limits or calculated from relative settings.
     * @return {north, east, rotation} Units are distance units or degrees per second.
     */
    double[] getVelocityCalculated();

    /**
     * Measured velocity based on motor or wheel encoders if present.
     * May be affected by acceleration limits or calculated from relative settings.
     * @return {north, east, rotation} Units are distance units or degrees per second.
     */
    double[] getVelocityMeasured();

    /**
     * Set maximum allowable acceleration.
     * Used to control wheelspin or lockup, depending on surface traction.
     * @param accelerationLimit In units of G (acceleration of gravity).
     * @param decellerationLimit In units of G (acceleration of gravity).
     */
    void setAccelerationLimits(double accelerationLimit, double decelerationLimit);

    /**
     * Acceleration/decelleration limits in units of G (acceleration of gravity).
     * @return {accelerationLimit, decelerationLimit}
     */
    double[] getAccelerationLimits();

}