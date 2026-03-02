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

        public static final int LEAD_MOTOR_ID = 14;
        public static final int FOLLOW_MOTOR_ID = 16;

        public static final boolean LEAD_MOTOR_IS_INVERTED = true;
        public static final boolean FOLLOW_MOTOR_INVERTED_FROM_LEAD = false;
    }

    public static final InterpolatingDoubleTreeMap DistanceToRPM = new InterpolatingDoubleTreeMap();
    static{
        DistanceToRPM.put(3.3, 0.8);
        DistanceToRPM.put(2.65, 0.8);
        DistanceToRPM.put(1.8, 1d);
        DistanceToRPM.put(2.1, 1d);
        DistanceToRPM.put(4.1, 1d);
    }

    public static double getRPM(Distance distance){
        return DistanceToRPM.get(distance.in(Meters));
    }
}
