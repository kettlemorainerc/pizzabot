package org.usfirst.frc.team2077.commands;

import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.*;
import org.usfirst.frc.team2077.drivetrain.*;
import org.usfirst.frc.team2077.sensors.*;

public class RotationMovement extends CommandBase {
    protected static final double MIN_ROTATION_LIMIT = .1;
    protected static final double ROTATION_LIMIT_RANGE = 1 - MIN_ROTATION_LIMIT;

    protected final DriveStick stick;
    protected final DriveChassisIF chassis;
    protected final AnalogSettings settings;

    public RotationMovement(RobotHardware hardware, DriveStick stick) {
        addRequirements(hardware.heading);

        this.stick = stick;
        this.chassis = hardware.chassis;
        this.settings = hardware.analogSettings;
    }

    @Override public void execute() {
        double limit = (ROTATION_LIMIT_RANGE * settings.get(AnalogSettings.Limit.ROTATION)) + MIN_ROTATION_LIMIT;

        chassis.setRotation01(stick.getRotation() * limit);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
