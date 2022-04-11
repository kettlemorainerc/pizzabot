/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * A command that's intended to be bound to some {@link JoystickButton Joystick button}
 */
public abstract class BindableCommand extends CommandBase {
    /**
     * Bind this command to a given joystick button
     * @param button a joystick button
     */
    public abstract void bind(JoystickButton button);

}
