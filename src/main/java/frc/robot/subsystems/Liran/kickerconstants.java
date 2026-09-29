// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Liran;

import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;

import com.revrobotics.spark.SparkLowLevel;


/** Add your docs here. */
public class kickerconstants {
    public static class LEADMOTOR {
        public static String NAME="leadmotor";
        public static  ControllerLocation LOCATION=null;
        public static boolean ISBRUSHLESS=true;
        public static int ID=56;
        public static final MotorType MOTOR_TYPE = edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType.;
        public static double SPEED=0.2;
    }

    public static class FUNNELMOTOR {
        public static String NAME="funnelmotor";
        public static ControllerLocation LOCATION=null;
        public static boolean ISBRUSHLESS=true;
        public static int ID=33;
        public static MotorType type=edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType.;
        public static double SPEED=0.19;
    }
    
}
