/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.sensors;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogSettings {

	public enum Limit {
		ACCELERATION(1),
		ROTATION(2),
		SPEED(3);

		public int INPUT_ID;

		Limit(int id) {
			this.INPUT_ID = id;
		}
	}

	private final Map<Integer,AnalogInput>  analogInput_ = new HashMap<>();

	public AnalogSettings() {
		for(Limit l : Limit.values()) analogInput_.put(l.INPUT_ID, new AnalogInput(l.INPUT_ID));
	}

	/** @return Analog input setting, scaled to range 0.0 - 1.0. */
	public double get(Limit l) {
		return analogInput_.get(l.INPUT_ID).getVoltage() / 5;
	}
}
