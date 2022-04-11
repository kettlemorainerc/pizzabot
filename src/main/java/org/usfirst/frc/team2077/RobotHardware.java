package org.usfirst.frc.team2077;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.drivetrain.*;
import org.usfirst.frc.team2077.sensors.*;
import org.usfirst.frc.team2077.subsystems.*;

public class RobotHardware {
    public final CANLineSubsystem.Pizza FRONT_LEFT_WHEEL = new CANLineSubsystem.Pizza(PizzabotDriveModule.ChassisPosition.FRONT_LEFT);
    public final CANLineSubsystem.Pizza FRONT_RIGHT_WHEEL = new CANLineSubsystem.Pizza(PizzabotDriveModule.ChassisPosition.FRONT_RIGHT);
    public final CANLineSubsystem.Pizza BACK_LEFT_WHEEL = new CANLineSubsystem.Pizza(PizzabotDriveModule.ChassisPosition.BACK_LEFT);
    public final CANLineSubsystem.Pizza BACK_RIGHT_WHEEL = new CANLineSubsystem.Pizza(PizzabotDriveModule.ChassisPosition.BACK_RIGHT);
    public final CANLineSubsystem.Pizza[] WHEELS = new CANLineSubsystem.Pizza[] {
        FRONT_LEFT_WHEEL, FRONT_RIGHT_WHEEL, BACK_LEFT_WHEEL, BACK_RIGHT_WHEEL
    };

//    public final CANLineSubsystem.SparkNeo SHOOTER = new CANLineSubsystem.Pizza(SparkNeoDriveModule.DrivePosition.SHOOTER);
//    public final CANLineSubsystem.Talon PRIMER = new CANLineSubsystem.Talon(6);
//    public final CANLineSubsystem.Talon OBTAINER = new CANLineSubsystem.Talon(7);

//    public final CANLineSubsystem.Talon CLIMBER_LEFT = new CANLineSubsystem.Talon(8);
//    public final CANLineSubsystem.Talon CLIMBER_RIGHT = new CANLineSubsystem.Talon(9);

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

    public CANLineSubsystem.Pizza getSparkNeoWheel(MecanumMath.WheelPosition position) {
        for(CANLineSubsystem.Pizza wheel : WHEELS) if(wheel.motor.getWheelPosition() == position) return wheel;

        return null;
    }
}
