/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static org.usfirst.frc.team2077.Robot.*;

import org.usfirst.frc.team2077.DriveStation;

public class AimCrosshairs extends CommandBase {

  public AimCrosshairs() {
    addRequirements(robot_.crosshairs_);
  }

  @Override
  public void execute() {
    double y = DriveStation.adjustInputSensitivity(-robot_.driveStation_.secondaryStick_.getY(), .2, 2.5);
    double x = DriveStation.adjustInputSensitivity(robot_.driveStation_.secondaryStick_.getX(), .2, 2.5);
    double[] crosshairs = robot_.crosshairs_.get();
    robot_.crosshairs_.set(crosshairs[0]+x, crosshairs[1]+y);
    //System.out.println(robot_.crosshairs_.toString());
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
