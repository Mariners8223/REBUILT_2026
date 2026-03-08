// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;

/** Add your docs here. */
public class KickerConstants {
    public static class MOTOR_CONSTANTS{
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final double GEAR_RATIO = 1;

        public static final int LEAD_MOTOR_CURRENT_LIMIT = 30;
        public static final int LEAD_MOTOR_CURRENT_THRESHOLD = 60;

        public static final int LEAD_MOTOR_ID = 16;
        public static final int FOLLOW_MOTOR_ID = 14;

        public static final boolean LEAD_MOTOR_IS_INVERTED = true;
        public static final boolean FOLLOW_MOTOR_INVERTED_FROM_LEAD = false;
    }

    public static final InterpolatingDoubleTreeMap DistantToDutyCycle = new InterpolatingDoubleTreeMap();
    static{
        DistantToDutyCycle.put(1.8, 1d);
        DistantToDutyCycle.put(3.3, 1d);
        DistantToDutyCycle.put(3.8, 0.8);
        DistantToDutyCycle.put(4.3, 0.8);
        DistantToDutyCycle.put(4.8, 0.6);
        DistantToDutyCycle.put(5.8, 0.6);
    }

    public static double getRPM(Distance distance){
        return DistantToDutyCycle.get(distance.in(Meters));
    }
}
