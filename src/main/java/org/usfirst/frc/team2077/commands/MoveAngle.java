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

import java.sql.Time;
import java.util.Timer;

import org.usfirst.frc.team2077.DriveStation;

public class MoveAngle extends CommandBase {
  private double finishTime;
  private double dps = 2; //degrees per second 
  private double angle_;


  // as is degrees/second
  public MoveAngle(double angle) {
    angle_ = angle;
    addRequirements(robot_.heading_);
  }

  @Override
  public void initialize() {
    finishTime = System.currentTimeMillis() + (angle_ / dps * 1000);
  }

  @Override
  public void execute() {
    //System.out.println(" STICK(1): " + clockwise);
    if (System.currentTimeMillis() < finishTime) {
      robot_.driveChassis_.setRotation(dps);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (System.currentTimeMillis() >= finishTime) {
      return true;
    } else {
      return false;
    }
  }


}
