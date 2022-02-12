/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static org.usfirst.frc.team2077.Robot.*;

import org.usfirst.frc.team2077.DriveStation;

public class PrimaryStickDrive extends CommandBase {

  public PrimaryStickDrive() {
    addRequirements(robot_.position_);
  }

  @Override
  public void execute() {
    // TODO: Check joystick capabilities.
    // TODO: Put deadband and exponent in drive station constants.
    double north = DriveStation.adjustInputSensitivity(-robot_.driveStation_.primaryStick_.getY(), .2, 2.5);
    double east = DriveStation.adjustInputSensitivity(robot_.driveStation_.primaryStick_.getX(), .2, 2.5);
    if (CommandScheduler.getInstance().requiring(robot_.heading_) != null) {
      // heading controlled elsewhere, just do position here
      //System.out.println(" STICK(3): " + north + " \t" + east);
      robot_.driveChassis_.setVelocity01(north, east);
    }
    else {
      // heading controlled here, do both position and rotation
      double clockwise = DriveStation.adjustInputSensitivity(robot_.driveStation_.primaryStick_.getTwist(), .2, 2.5);
      //System.out.println(" STICK(2): " + north + " \t" + east + " \t" + clockwise);
      robot_.driveChassis_.setVelocity01(north, east, clockwise);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
