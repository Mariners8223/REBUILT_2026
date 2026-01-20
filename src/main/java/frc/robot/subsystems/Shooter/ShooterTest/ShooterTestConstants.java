// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.ShooterTest;
import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;


/** Add your docs here. */
public class ShooterTestConstants {

    public static class Motor1{
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;//TODO:probably change
        public static final int MOTOR_ID = 1;

        public static final boolean IS_INVERTED = true;

        public static final PIDFGains PID_GAINS = new PIDFGains(
            200,2,0,0,0.0,0.05);//TODO:change
        public static final double GEAR_RATION = 1;//TODO:change
    }

    public static class Motor2{
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;//TODO:probably change
        public static final int MOTOR_ID = 2;

        public static final boolean IS_INVERTED = false;

        public static final PIDFGains PID_GAINS = new PIDFGains(
            200,2,0,0,0.0,0.05);//TODO:change
        public static final double GEAR_RATION = 1;//TODO:change
    }
}
