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

public class SecondaryStickDrive extends CommandBase {

  public SecondaryStickDrive() {
    addRequirements(robot_.heading_);
  }

  @Override
  public void execute() {
    // TODO: Check joystick capabilities.
    // TODO: Put deadband and exponent in drive station constants.
    double clockwise = DriveStation.adjustInputSensitivity(robot_.driveStation_.secondaryStick_.getX(), .2, 2.5);
    //System.out.println(" STICK(1): " + clockwise);
    robot_.driveChassis_.setRotation01(clockwise);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
