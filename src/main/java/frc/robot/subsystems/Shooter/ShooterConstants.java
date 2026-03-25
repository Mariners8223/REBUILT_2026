// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;

/** Add your docs here. */
public class ShooterConstants {
    public static class MOTOR_CONSTANTS{
        public static final double Ks = 0.0060295;
        public static final double Kv = 0.11755;
        public static final double Ka = 0.012639;

        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final PIDFGains PID = new PIDFGains(
            0.4,
            0,
            0.07,
            Kv
        );

        public static final double GEAR_RATIO = 1;
        public static final double GEAR_REDUCTION = 1 / GEAR_RATIO;

        public static final int SHOOTER_MOTOR_CURRENT_LIMIT = 30;
        public static final int SHOOTER_MOTOR_CURRENT_THRESHOLD = 60;

        public static final int LEAD_MOTOR_ID = 5;
        public static final boolean LEAD_MOTOR_IS_INVERTED = false;

        public static final int FOLLOW_MOTOR_ID = 21;
        public static final boolean IS_INVERTED_FROM_LEADER = true;
    }

    public static final AngularVelocity PASSING_VELOCITY = RPM.of(4000);
    public static final AngularAcceleration FALL_ACCELERATION = RotationsPerSecondPerSecond.of(30);
    public static final AngularVelocity SHOOTER_ANGULAR_VELOCITY_TOLERANCE = RPM.of(100);

    public static final Distance SHOOTER_WHEEL_RADIUS = Centimeter.of(10);
    public static final Distance SHOOTER_WHEEL_CIRCUMFERENCE = SHOOTER_WHEEL_RADIUS.times(2 * Math.PI);
    public static final Mass SHOOTER_WHEEL_MASS = Kilogram.of(0.5);
    public static final MomentOfInertia SHOOTER_MOMENT_OF_INERTIA = KilogramSquareMeters.of(1);

    public static class Maps{
        public static final InterpolatingDoubleTreeMap DistanceToRPM = new InterpolatingDoubleTreeMap();

        static{
            DistanceToRPM.put(1.8, 2500.0);
            DistanceToRPM.put(2.3, 2630.0);
            DistanceToRPM.put(2.8, 2790.0);
            DistanceToRPM.put(3.3, 2990.0);
            DistanceToRPM.put(3.8, 3240.0);
            DistanceToRPM.put(4.3, 3500.0);
            DistanceToRPM.put(4.8, 3630.0);
            DistanceToRPM.put(5.3, 3700.0);
            DistanceToRPM.put(5.8, 3870.0);
        }
    }

    public static AngularVelocity requiredAngularVelocity(Distance distance){
        return RPM.of(Maps.DistanceToRPM.get(distance.in(Meters)));
    }
}
