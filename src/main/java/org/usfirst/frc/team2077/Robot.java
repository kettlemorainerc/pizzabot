/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.sensors.*;

// AJ- Changes
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// AJ- Changes

// The default Command template from WPI splits a lot of top level declaration
// and initialization into a separate RobotContainer class. The main purpose of
// doing so seems to be to make the code less convenient to read :)
// In this example such code has been merged back into this file. - RAB

// The following header comment from the template file is just plain wrong.
// It applies to frc.robot.Main. Probably a cut/paste error as code evolved. - RAB

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    public static Robot robot;
    // Drive station controls.
    public DriveStation driveStation_;
    // Inter-process data exchange.
    public NetworkTableInstance networkTableInstance;
    // Default commands
    //    Autonomous selected via drive station dashboard.
    protected Command autonomous_;
    private RobotHardware hardware;
    int sentinel = 0;

    public Robot() {
        robot = this;
    }

    /**
     * Run once on startup.
     */
    @Override
    public void robotInit() {
        hardware = new RobotHardware();
        // Container for remote control software objects.
        driveStation_ = new DriveStation(hardware);

        // Communications with other processes on the DS.
        networkTableInstance = NetworkTableInstance.getDefault();

        // Test code.
        //driveStation_.primary4_.whenPressed();
        //driveStation_.primary5_.whenPressed();  // TODO: debug
        //driveStation_.primary6_.whenPressed();
        //driveStation_.primary7_.whenPressed();  // TODO: debug
        //driveStation_.primary8_.whenPressed();
        //driveStation_.primary9_.whenPressed();  // TODO: debug
        //driveStation_.secondaryTrigger_.whenPressed(new ToggleLauncher());
        //driveStation_.secondary5_.whileHeld(new LoadLauncher());  // TODO: debug
        //driveStation_.secondary6_.whileHeld(new AngleLauncher());
        //driveStation_.primaryTrigger_.whileHeld(new RunGrabber());
        //driveStation_.primary8_.whenPressed(new MoveAngleTarget());
        //driveStation_.secondaryTrigger_.whileActiveContinuous(new ContinousAimToTarget3());
        //driveStation_.primary12_.whileHeld(new ToggleLauncher(), true);


//    (new JoystickButton(driveStation_.primaryStick_, 7)).whenPressed(new Rotate(90));
//    (new JoystickButton(driveStation_.primaryStick_, 8)).whenPressed(new Rotate(-90));
//    (new JoystickButton(driveStation_.primaryStick_, 9)).whenPressed(new Rotate(360));
//    (new JoystickButton(driveStation_.primaryStick_, 10)).whenPressed(new Rotate(-360));
//    (new JoystickButton(driveStation_.primaryStick_, 11)).whenPressed(new Rotate(1080));
//    (new JoystickButton(driveStation_.primaryStick_, 12)).whenPressed(new Rotate(-1080));


        //(new JoystickButton(driveStation_.secondaryStick_, 6)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getVoltageTest());
        //(new JoystickButton(driveStation_.secondaryStick_, 7)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getRatioTest());
        //(new JoystickButton(driveStation_.secondaryStick_, 8)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getRevolutionsTest(10, .2));
        //(new JoystickButton(driveStation_.secondaryStick_, 6)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getVelocityTest(32, .5));
        //(new JoystickButton(driveStation_.secondaryStick_, 7)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getVelocityTest(16, 1));
        //(new JoystickButton(driveStation_.secondaryStick_, 8)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getVelocityTest(8, 2));
        //(new JoystickButton(driveStation_.secondaryStick_, 9)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getVelocityTest(4, 3));
        //(new JoystickButton(driveStation_.secondaryStick_, 10)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getVelocityTest(2, 4));
        //(new JoystickButton(driveStation_.secondaryStick_, 11)).whenPressed(((PizzabotDriveModule)pizzaDriveModules[0]).getVelocityTest(1, 8));
    }

    /**
     * Called every robot packet (generally about 50x/second) no matter the mode. Use this for items like diagnostics
     * that you want run during disabled, autonomous, teleoperated and test.
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated
     * updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance()
                        .run();
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
     * Called once each time the robot enters disabled mode. Note that in competition the robot may (or may not?) be
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

//    autonomous_ = new AutonomousOperations();

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
        CommandScheduler.getInstance()
                        .cancelAll();

    }

    /**
     * Called periodically during test.
     */
    @Override
    public void testPeriodic() {

    }
}
