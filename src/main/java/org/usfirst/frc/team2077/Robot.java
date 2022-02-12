/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import org.usfirst.frc.team2077.commands.AimCrosshairs;
import org.usfirst.frc.team2077.commands.PrimaryStickDrive;
import org.usfirst.frc.team2077.commands.SecondaryStickDrive;
import org.usfirst.frc.team2077.commands.SteerToCrosshairs;
import org.usfirst.frc.team2077.commands.MoveAngle;
import org.usfirst.frc.team2077.drivetrain.DifferentialChassis;
import org.usfirst.frc.team2077.drivetrain.DriveChassis;
import org.usfirst.frc.team2077.drivetrain.DriveModule;
import org.usfirst.frc.team2077.drivetrain.DummyDriveModule;
import org.usfirst.frc.team2077.drivetrain.MecanumChassis;
import org.usfirst.frc.team2077.drivetrain.SparkNeoDriveModule;
import org.usfirst.frc.team2077.drivetrain.PizzabotDriveModule;
import org.usfirst.frc.team2077.subsystems.Crosshairs;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// The default Command template from WPI splits a lot of top level declaration
// and initialization into a separate RobotContainer class. The main purpose of
// doing so seems to be to make the code less convenient to read :)
// In this example such code has been merged back into this file. - RAB

// The following header comment from the template file is just plain wrong.
// It applies to frc.robot.Main. Probably a cut/paste error as code evolved. - RAB

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Using a constructed constants object instead of statics.
  // Put globally accessible system constants in the Constants class.
  // Other code can access them through Robot.robot_.constants_.<FIELD_NAME>.
  public Constants constants_ = new Constants();

  // Other globally accessible objects...

  // Drive station controls.
  public DriveStation driveStation_;

  // Sensors.
  public AnalogInput testAnalogSensor_;

  // Drive train, including:
  //   Controller/motor/wheel/encoder units for each wheel.
  //   Logic for applying robot level functionality to individual wheels.
  public DriveChassis driveChassis_;

  // Subsystems
  //    The position/heading subsystems operate as flags to allow control
  //    of chassis rotation to move between commands independently of positioning.
  public Subsystem position_;
  public Subsystem heading_;
  //    Aiming system for elevating shooter and pointing the robot. Displayed on DS video.
  public Crosshairs crosshairs_;

  // Commands
  //    Autonomous selected via drive station dashboard.
  private Command autonomous_;
  //    Default teleop robot drive.
  private Command drive_;
  //    Point shooter to target by rotating robot and elevating barrel.
  private Command aim_;

  // Everything "global" hangs off the single instance of Robot,
  // either directly or under one of the above public members.
  public static Robot robot_;
  // This class will be instantiated exactly once, via frc.robot.Main.
  // The constructor initializes the globally accessible static instance,
  // all other initialization happens in robotInit().
  public Robot() {
    robot_ = this;
  }

  /**
   * Run once on startup.
   */
  @Override
  public void robotInit() {

    // Container for remote control software objects.
    driveStation_ = new DriveStation();

    // Test stuff.
    testAnalogSensor_ = new AnalogInput(0);

    // Drivetrain.
//    driveChassis_ = new DifferentialChassis(new DriveModule[] {
//      new SparkNeoDriveModule(1, true),
//      new SparkNeoDriveModule(2, false),
//      new DummyDriveModule(),  // northeast (right front)
//      new DummyDriveModule()   // nortwhest (left front)
//    //}, constants_.WHEELBASE, constants_.TRACK_WIDTH, constants_.WHEEL_RADIUS);
//    }, constants_.WHEELBASE, constants_.TRACK_WIDTH, constants_.WHEEL_RADIUS);
  driveChassis_ = new MecanumChassis(new DriveModule[] {
      new PizzabotDriveModule(1, 10, 11, false),  // northeast (right front)
      new PizzabotDriveModule(2, 12, 13, false),  // southeast (right rear)
      new PizzabotDriveModule(3, 18, 19, true),  // southwest (left rear)
      new PizzabotDriveModule(4, 20, 21, true)   // nortwhest (left front)
    }, constants_.WHEELBASE, constants_.TRACK_WIDTH, constants_.WHEEL_RADIUS);

    // Subsystems.
    //   These dummy subsystems support seperable command ownership of robot motion and rotation.
    position_ = new SubsystemBase() {};
    heading_ = new SubsystemBase() {};

    //   Manages an operator-controlled aiming point displayed on DS and used for targeting.
    crosshairs_ = new Crosshairs(640, 360);

    // Default teleop commands.
    drive_ = new PrimaryStickDrive();
    aim_ = new AimCrosshairs();
    driveStation_.primary10.whenPressed(new MoveAngle(180));

    // TODO: There's supposedly another way to do this.
    CommandScheduler.getInstance().setDefaultCommand(position_, drive_);
    // Uncomment the following to move rotation to secondary stick.
    //CommandScheduler.getInstance().setDefaultCommand(heading_, new SecondaryStickDrive());

    // Aiming system work in progress.
    //CommandScheduler.getInstance().setDefaultCommand(heading_, new SteerToCrosshairs());
    CommandScheduler.getInstance().setDefaultCommand(crosshairs_, aim_);
  }

  /**
   * Called every robot packet (generally about 50x/second) no matter the mode.
   * Use this for items like diagnostics that you want run during disabled,
   * autonomous, teleoperated and test.
   * <p>
   * This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Sensor testing...
    //System.out.println("Analog 0: " + testAnalogSensor_.getValue() + " (" + testAnalogSensor_.getVoltage() + "V)");
  }

  // The robot and the drive station exchange data packets around 50x/second so long
  // as they are connected and the robot program is running (hasn't crashed or exited).
  // This packet excahange is what keeps the DS related software objects, i.e. Joysticks,
  // in the robot code updated with their position, etc on the the actual DS, and what
  // keeps "Robot Code" indicator on the DS green.
  // 
  // Each time a DS packet is received, the underlying WPILIB code calls one or more
  // xxxPeriodic() methods in this class, first a mode-specific one and then robotPeriodic().
  //
  // Each time the robot mode (disabled, autonomous, teleop, test) changes the appropriate
  // xxxInit() method is called. The robotInit() method is called only once at startup.

  /**
   * Called once each time the robot enters disabled mode.
   * Note that in competition the robot may (or may not?) be
   * disabled briefly between autonomous and teleop.
   */
  @Override
  public void disabledInit() {
  }

  /**
   * Called periodically while robot is disabled.
   */
 @Override
  public void disabledPeriodic() {
  }

  /**
   * Called once each time the robot enters autonomous mode.
   */
  @Override
  public void autonomousInit() {
  
    autonomous_ = null; // TODO: This is a placeholder

    // schedule the autonomous command (example)
    if (autonomous_ != null) {
      autonomous_.schedule();
    }
  }

  /**
   * Called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * Called once each time the robot enters teleop (operator controlled) mode.
   */
  @Override
  public void teleopInit() {

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomous_ != null) {
      autonomous_.cancel();
    }
  }

  /**
   * Called periodically during teleop.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * Called once each time the robot enters test mode.
   */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * Called periodically during test.
   */
  @Override
  public void testPeriodic() {
  }
}
