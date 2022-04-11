/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.*;
import org.usfirst.frc.team2077.drivetrain.*;
import org.usfirst.frc.team2077.sensors.*;

public class CardinalMovement extends CommandBase {
	private static final double MIN_SPEED_LIMIT = .1;
	private static final double SPEED_LIMIT_RANGE = 1. - MIN_SPEED_LIMIT;

	public static final double ACCELERATION_G_LIMIT = .4;
	public static final double DECELERATION_G_LIMIT = ACCELERATION_G_LIMIT; //1e10 //.35 is the value used for the 03-05-21 version

	protected final DriveStick stick;
	protected final DriveChassisIF chassis;
	private final AnalogSettings settings;

	public CardinalMovement(RobotHardware hardware, DriveStick stick) {
		addRequirements(hardware.position);

		this.stick = stick;
		this.chassis = hardware.chassis;
		this.chassis.setGLimits(ACCELERATION_G_LIMIT, DECELERATION_G_LIMIT);
		this.settings = hardware.analogSettings;
	}

	@Override public void execute() {
		double speedLimit = (SPEED_LIMIT_RANGE * settings.get(AnalogSettings.Limit.SPEED)) + MIN_SPEED_LIMIT;

		double north = -stick.getNorth();
		double east = stick.getEast();

		// Tank drive
//		north = Math.abs(north) >= Math.abs(east) ? north : 0;
//		east = Math.abs(east) > Math.abs(north) ? east : 0;

		chassis.setVelocityPercent(north * speedLimit, east * speedLimit);
	}

	@Override public void end(boolean interrupted) {}

	@Override public boolean isFinished() {
		return false;
	}
}
