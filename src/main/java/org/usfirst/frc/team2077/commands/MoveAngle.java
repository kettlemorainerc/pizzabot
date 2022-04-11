/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import static org.usfirst.frc.team2077.Robot.*;

import java.sql.Time;
import java.util.Timer;

import org.usfirst.frc.team2077.*;
import org.usfirst.frc.team2077.drivetrain.*;

public class MoveAngle extends CommandBase {
  private double finishTime;
  private double dps_;
  private double angle_;
  private double distance;
  private double finalHeading;

  private final AbstractChassis chassis;

  // as is degrees/second
  
  public MoveAngle(RobotHardware hardware, double angle) {
    this(hardware, angle, 5);
    System.out.println("**************" + angle + "****************");
  }

  public MoveAngle(RobotHardware hardware, double angle, double dps) {
    this.chassis = hardware.chassis;
    angle_ = angle;
    dps_ = dps;
    addRequirements(hardware.heading);
  }

  @Override
  public void initialize() {

    finalHeading = chassis.getPosition().get(MecanumMath.VelocityDirection.ROTATION) + angle_;
    //distance = constants_.TESTBOT_TRACK_WIDTH;
  }

  @Override
  public void execute() {
    //System.out.println("###############################################");
    if (angle_ < 0) {
      if (chassis.getPosition().get(MecanumMath.VelocityDirection.ROTATION) > finalHeading) {
        chassis.setRotation(- dps_);
      }
    } else {
      if (chassis.getPosition().get(MecanumMath.VelocityDirection.ROTATION) < finalHeading) {
        chassis.setRotation(dps_);
      } 
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    if (angle_ > 0) {
      if (chassis.getPosition().get(MecanumMath.VelocityDirection.ROTATION) >= finalHeading) {
        return true;
      }
    } else { //rotation is negative
      if (chassis.getPosition().get(MecanumMath.VelocityDirection.ROTATION) <= finalHeading) {
        return true;
      }
    }
    return false;
  }
}
