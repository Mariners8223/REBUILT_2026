// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
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

    public static class HUB_CONSTANTS{
        public static Distance HUB_HEIGHT = Inches.of(72);
        public static Distance HUB_INSCRIBED_CIRCLE_DIAMETER = Inches.of(41.7);
        public static Distance HUB_CIRCUMSCRIBED_CIRCLE_DIAMETER = Inches.of(47);
    }

    public static double LOW_PASS_FILTER_ALPHA = 0.3;
    public static double FEED_FORWARD_SHOOTER_BOOST = 0.06;
    public static double MAX_FEED_FORWARD_BOOST = 5;
    public static AngularVelocity SHOOTING_VELOCITY_FALL = RotationsPerSecond.of(2);
    public static final AngularAcceleration FALL_ACCELERATION = RotationsPerSecondPerSecond.of(30);
    public static Time FEED_FORWARD_BOOST_TIME = Millisecond.of(80);

    public static final Distance SHOOTER_HEIGHT = Centimeter.of(40);
    public static final Angle SHOOTER_ANGLE = Degree.of(75);
    public static final LinearAcceleration ACCELERATION_DUE_TO_GRAVITY = MetersPerSecondPerSecond.of(9.8);


    public static final Distance SHOOTER_WHEEL_RADIUS = Centimeter.of(10);
    public static final Distance SHOOTER_WHEEL_CIRCUMFERENCE = SHOOTER_WHEEL_RADIUS.times(2 * Math.PI);
    public static final Mass SHOOTER_WHEEL_MASS = Kilogram.of(0.5);
    public static final MomentOfInertia SHOOTER_MOMENT_OF_INERTIA = KilogramSquareMeters.of(1);

    public static final AngularVelocity SHOOTER_ANGULAR_VELOCITY_TOLERANCE = RPM.of(100);



    public static final Distance KICKER_WHEEL_RADIUS = Centimeter.of(10);
    public static final Distance KICKER_WHEEL_CIRCUMFERENCE = KICKER_WHEEL_RADIUS.times(2 * Math.PI);
    public static final double KICKER_MOMENT_OF_INERTIA = 0;

    public static final Voltage KICKER_ACTIVE_VOLTAGE = Volts.of(3);


    public static final Pose3d POSITION = new Pose3d(
        new Translation3d(0, 0, 0), new Rotation3d(0, 0, 0)
    );
    public static final double PASSING_VELOCITY = 0;

    public static class Maps{
        public static final InterpolatingDoubleTreeMap DistanceToRPM = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap DistanceToRPMPassing = new InterpolatingDoubleTreeMap();

        public static final InterpolatingDoubleTreeMap DistanceToFlytime = new InterpolatingDoubleTreeMap();

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

            DistanceToRPMPassing.put(4.0, 3000.0);
            DistanceToRPMPassing.put(6.2, 4000.0);
            DistanceToRPMPassing.put(6.8, 5000.0);
        }
    }

    public static class Calculations{
        // public static Distance practicalDistance(Translation2d hubPose, Pose2d robotPose, double velocityX, double velocityY){
        //     Translation2d hubToRobot = hubPose.minus(robotPose.getTranslation());
        //     double distance = hubToRobot.getNorm();

        //     Translation2d velocity = new Translation2d(velocityX, velocityY);
        //     double flightTime = LookupTable.DISTANCE_TO_TIME_OF_FLIGHT.get(distance);
        //     Translation2d displacementFromRobotVelocity = velocity.times(flightTime);

        //     hubToRobot.minus(displacementFromRobotVelocity);

        //     return Meters.zero();
        // }

        // public static LinearVelocity requiredLinearVelocity(Distance distance){
        //     /*  h = y0 + tan(alpha) * d - (g / (2 * (v * cos(alpha))^2)) * d^2
        //         h - Hub Height
        //         d - Distance from centre of Hub
        //         y0 - Height of Shooter from ground
        //         alpha - Launch Angle
        //         g - Acceleration due to Gravity

        //         v - Launch Velocity

        //         Solve for v:
        //         v^2 = g*d^2 / (2 * cos(alpha) * (x*sin(alpha) - (h-y0)*cos(alpha)))
        //     */

        //     double cos = Math.cos(SHOOTER_ANGLE.in(Radian));
        //     double sin = Math.sin(SHOOTER_ANGLE.in(Radian));
        //     double height_differential = (HUB_CONSTANTS.HUB_HEIGHT.minus(SHOOTER_HEIGHT)).in(Meter);

        //     double numerator = ACCELERATION_DUE_TO_GRAVITY.baseUnitMagnitude() * (distance.times(distance)).baseUnitMagnitude();
        //     double denominator = 2 * cos * (distance.baseUnitMagnitude() * sin - height_differential * cos);
        //     double final_solution = Math.sqrt(numerator / denominator);

        //     return MetersPerSecond.of(final_solution);
        // }

        // public static AngularVelocity requiredAngularVelocity(LinearVelocity launchVelocity){
        //     return RotationsPerSecond.of(launchVelocity.in(Meters.per(Minute)) / SHOOTER_WHEEL_RADIUS.in(Meter)); // TODO: MAKE MORE EXACT FUNCTION
        // }

        public static AngularVelocity requiredAngularVelocity(Distance distance){
            return RPM.of(Maps.DistanceToRPM.get(distance.in(Meters)));
        }

        public static <U extends Unit> boolean epsilonEquals(Measure<U> unit1, Measure<U> unit2, Measure<U> tolerance){
            return Math.abs(unit1.baseUnitMagnitude() - unit2.baseUnitMagnitude()) <= tolerance.baseUnitMagnitude();
        }
    }
}
