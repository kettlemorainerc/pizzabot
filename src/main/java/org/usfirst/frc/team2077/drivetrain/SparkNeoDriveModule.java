

package org.usfirst.frc.team2077.drivetrain;


import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class SparkNeoDriveModule extends CANSparkMax implements DriveModule {
    //6 inch wheels on rnd bot
    private CANPIDController pidController;
    private CANEncoder encoder;
    private double setPoint;
    private double circumference = 12 * Math.PI; //2*pi*radius radius = 6
    private double maxRPM = 5000;
    private double gearRatio = 12.75;
    private boolean isReverse;
    

    public SparkNeoDriveModule(int deviceID, boolean isReverse_) {
        super(deviceID, MotorType.kBrushless);
        pidController = this.getPIDController();
        encoder = this.getEncoder();
        isReverse = isReverse_;
        pidController.setP(5e-5);
        pidController.setI(1e-6);
        pidController.setD(0);
        pidController.setIZone(0);
        pidController.setFF(0);
        pidController.setOutputRange(-1, 1);


    }
    

    /**
     * Set velocity for this wheel.
     * @param velocity In inches/second.
     * Positive values are robot-forward ("north"), negative backward/south.
     */
    public void setVelocity(double velocity) {
        //convert from inches/second to rpm
        setPoint = velocity*gearRatio*60/circumference;
        if (setPoint > maxRPM) {
            setPoint = maxRPM;
        }
        if (isReverse) {
            pidController.setReference(-setPoint, ControlType.kVelocity);
        } else {
            pidController.setReference(setPoint, ControlType.kVelocity);
        }
    }


    /**
     * Current velocity for this wheel.
     * This should be a direct measurement from an encoder if available,
     * otherwise the set point as passed to {@link #setVelocity} or reported by
     * the motor controller.
     * @return Velocity In inches/second.
     */
    public double getVelocity() {
        double velocity = encoder.getVelocity()/60/gearRatio*circumference; //need to still convert to inches per second
        return velocity;
    }


    /**
     * Distance traveled by the wheel since startup or the last reset.
     * This should be a direct measurement from an encoder if available,
     * otherwise computed by integrating velocity over time.
     * @return Distance in inches.
     */
    public double getDistance() {
        return encoder.getPosition()/gearRatio*circumference;
    }

    /**
     * Reset the distance measurement to zero inches.
     */
    public void resetDistance() {
        encoder.setPosition(0);
    }
}