package org.usfirst.frc.team2077;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.drivetrain.*;
import org.usfirst.frc.team2077.sensors.*;
import org.usfirst.frc.team2077.subsystems.*;

public class RobotHardware {
    public final CANLineSubsystem.SparkNeo FRONT_LEFT_WHEEL = new CANLineSubsystem.SparkNeo(SparkNeoDriveModule.DrivePosition.FRONT_LEFT);
    public final CANLineSubsystem.SparkNeo FRONT_RIGHT_WHEEL = new CANLineSubsystem.SparkNeo(SparkNeoDriveModule.DrivePosition.FRONT_RIGHT);
    public final CANLineSubsystem.SparkNeo BACK_LEFT_WHEEL = new CANLineSubsystem.SparkNeo(SparkNeoDriveModule.DrivePosition.BACK_LEFT);
    public final CANLineSubsystem.SparkNeo BACK_RIGHT_WHEEL = new CANLineSubsystem.SparkNeo(SparkNeoDriveModule.DrivePosition.BACK_RIGHT);
    public final CANLineSubsystem.SparkNeo[] WHEELS = new CANLineSubsystem.SparkNeo[] {
        FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, BACK_LEFT_WHEEL, BACK_RIGHT_WHEEL
    };

    public final CANLineSubsystem.SparkNeo SHOOTER = new CANLineSubsystem.SparkNeo(SparkNeoDriveModule.DrivePosition.SHOOTER);
    public final CANLineSubsystem.Talon PRIMER = new CANLineSubsystem.Talon(6);
    public final CANLineSubsystem.Talon OBTAINER = new CANLineSubsystem.Talon(7);

    public final CANLineSubsystem.Talon CLIMBER_LEFT = new CANLineSubsystem.Talon(8);
    public final CANLineSubsystem.Talon CLIMBER_RIGHT = new CANLineSubsystem.Talon(9);

    public final Subsystem heading = new Subsystem() {};
    public final Subsystem position = new Subsystem() {};

    public final AbstractChassis chassis;

    public final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    public final AHRS navX = new AHRS(SPI.Port.kMXP, (byte)100);
    public final AngleSensor angleSensor;

    public final PowerDistribution pdh = new PowerDistribution();

    public final AnalogSettings analogSettings = new AnalogSettings();


    public RobotHardware() {
        angleSensor = new AngleSensor(this);
        chassis = new MecanumChassis(this);
    }

    public CANLineSubsystem.SparkNeo getSparkNeoWheel(MecanumMath.WheelPosition position) {
        for(CANLineSubsystem.SparkNeo wheel : WHEELS) if(wheel.motor.getWheelPosition() == position) return wheel;

        return null;
    }
}
