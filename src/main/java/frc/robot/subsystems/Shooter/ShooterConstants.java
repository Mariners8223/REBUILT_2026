// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;

/** Add your docs here. */
public class ShooterConstants {

    public static class SHOOTER_MOTOR{
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final PIDFGains PID = new PIDFGains(
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0
        );
        public static final double GEAR_RATIO = 0;

        public static final int MOTOR_1_ID = 0;
        public static final int MOTOR_2_ID = 0;

        public static final boolean MOTOR_1_IS_INVERTED = false;
        public static final boolean MOTOR_2_IS_INVERTED = false;
    }

    public static class KICKER_MOTOR{
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final PIDFGains PID = new PIDFGains(
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0
        );
        public static final double GEAR_RATIO = 0;

        public static final int MOTOR_1_ID = 0;
        public static final int MOTOR_2_ID = 0;

        public static final boolean MOTOR_1_IS_INVERTED = false;
        public static final boolean MOTOR_2_IS_INVERTED = false;
    }

    public static class HUB_CONSTANTS{
        public static Distance HUB_HEIGHT = Inches.of(72);
        public static Distance HUB_INSCRIBED_CIRCLE_DIAMETER = Inches.of(41.7);
        public static Distance HUB_CIRCUMSCRIBED_CIRCLE_DIAMETER = Inches.of(47);
    }

    public static final Distance SHOOTER_WHEEL_RADIUS = Centimeter.of(10);
    public static final Distance SHOOTER_WHEEL_CIRCUMFERENCE = SHOOTER_WHEEL_RADIUS.times(2 * Math.PI);
    public static final Mass SHOOTER_WHEEL_MASS = Kilogram.of(0.5);
    public static final Angle SHOOTER_ANGLE = Degree.of(75);


    public static final Distance KICKER_WHEEL_RADIUS = Centimeter.of(10);
    public static final Distance KICKER_WHEEL_CIRCUMFERENCE = KICKER_WHEEL_RADIUS.times(2 * Math.PI);

    public static final Pose3d POSITION = new Pose3d(
        new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)
    );

    public static class Calculations{
        public double requiredLinearVelocity(double distance){
            
        }
    }
}
