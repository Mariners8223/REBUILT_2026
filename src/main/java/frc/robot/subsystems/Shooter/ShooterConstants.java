// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Kilogram;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

/** Add your docs here. */
public class ShooterConstants {

    public static class ShooterMotor{
        public static final int motorID = 0;
    }

    public static class TriggerMotor{
        public static final int motorID = 0;
    }

    public static final Distance shooterWheelRadius = Centimeter.of(10);
    public static final Distance shooterWheelCircumference = shooterWheelRadius.times(2 * Math.PI);
    public static final Mass shooterWheelWeight = Kilogram.of(0.5);


    public static final Distance triggerWheelRadius = Centimeter.of(10);
    public static final Distance triggerWheelCircumference = shooterWheelRadius.times(2 * Math.PI);
}
