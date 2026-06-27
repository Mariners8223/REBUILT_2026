// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;

/** Add your docs here. */
public class PivotConstants {
    public enum PivotStates
    {
        Closed(Degrees.of(0)),
        Middle(Degrees.of(-45)),
        Open(Degrees.of(-90));

        private final Angle angle;

        PivotStates(Angle angle)
        {
            this.angle = angle;
        }

        public Angle getAngle()
        {
            return this.angle;
        }
    }

    public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
    public static final int MOTOR_ID = 56;
    public static final boolean IS_BRUSHLESS = true;
    public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
    public static final boolean IS_INVERTED = true;
    public static final double GEAR_RATIO = 60;

    public static final double SOFT_MINIMUM = PivotStates.Open.getAngle().minus(Degrees.of(5)).in(Rotations);
    public static final double SOFT_MAXIMUM = PivotStates.Closed.getAngle().plus(Degrees.of(5)).in(Rotations);
    public static final PivotStates RESET = PivotStates.Closed;

    public static final PIDFGains PID_GAINS = new PIDFGains(
    250, // was 600 but its really violent so i lowered it a bit | 24/06 was 500 but still very violent so 250 it is
    25,
    0,
    0.1);

    public static final double STALL_CURRENT = 40;
}
