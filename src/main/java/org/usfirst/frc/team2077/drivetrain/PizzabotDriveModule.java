/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class PizzabotDriveModule implements DriveModule {

    // velocity set point
    private double velocity_ = 0;

    // accumulated distance since last velocity change or distance reset
    private double distance_ = 0;

    // time of last distance reset
    private long distanceTime_ = System.currentTimeMillis();

    private final boolean reversed_;

    private final WPI_TalonSRX  talonSRX_;

    private final Encoder wheelEncoder_;

    private static final double MOTOR_WHEEL_RATIO = 21.25;
    private static final boolean INVERTING_GEARBOX = false;
    private static final int ENCODER_PULSES_PER_REVOLUTION = 2048; // pulses per revolution (for CUI AMT103 this is the switch setting per their data sheet
    private static final int ENCODER_COUNTS_PER_REVOLUTION = (int)Math.round(4 * ENCODER_PULSES_PER_REVOLUTION * MOTOR_WHEEL_RATIO); 
    private static final double motorKP = 0.03;//0.6);
    private static final double motorKI = 0.00;//0.001);
    private static final double motorKD = 0.00;
    private static final double motorKF = 1023/(3400 * MOTOR_WHEEL_RATIO);

    private static final double velocityConversion_ = 600; // TODO: Fix me.

    // TODO: Units, units, and more units.

    public PizzabotDriveModule(int canID, int wheelEncoderChannelA, int wheelEncoderChannelB, boolean reversed) {

        talonSRX_ = new WPI_TalonSRX(canID);
        talonSRX_.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0); // TODO: return code? timeout value?
        talonSRX_.configSelectedFeedbackCoefficient(1.0, 0, 0); // TODO: return code? timeout value?
        talonSRX_.setSensorPhase(INVERTING_GEARBOX);
        talonSRX_.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10, 0); // TODO: frame? timeout value?
        talonSRX_.config_kP(0, motorKP, 0);
        talonSRX_.config_kI(0, motorKI, 0);
        talonSRX_.config_kD(0, motorKD, 0);
        talonSRX_.config_kF(0, motorKF, 0);
        talonSRX_.configMaxIntegralAccumulator(0, 300, 0); // TODO: ????????????
        talonSRX_.configClosedloopRamp(0.2, 0);  // .2 seconds from neutral to full
        talonSRX_.setNeutralMode(NeutralMode.Brake);

        wheelEncoder_ = new Encoder(new DigitalInput(wheelEncoderChannelA), new DigitalInput(wheelEncoderChannelB), false, EncodingType.k4X);
        wheelEncoder_.setDistancePerPulse(4);

        reversed_ = reversed;
    }

    @Override
    public void setVelocity(double velocity) {
        long now = System.currentTimeMillis();
        distance_ += velocity_ * ((now-distanceTime_)/1000.);
        distanceTime_ = now;
        velocity_ = velocity;
        
        talonSRX_.set(ControlMode.Velocity, velocityConversion_ * velocity_ * (reversed_ ? -1 : 1));
    }

    @Override
    public double getVelocity() {
        //return velocity_;

        //return talonSRX_.getSensorCollection().getQuadratureVelocity() * (reversed_ ? -1 : 1);

        return wheelEncoder_.getRate() *MOTOR_WHEEL_RATIO / 10; 
    }

    @Override
    public double getDistance() {
        long now = System.currentTimeMillis();
        //return distance_ + velocity_ * ((now-distanceTime_)/1000.);

        return wheelEncoder_.getDistance();
    }

    @Override
    public void resetDistance() {
        distance_ = 0;
        distanceTime_ = System.currentTimeMillis();

        wheelEncoder_.reset();
    }
}