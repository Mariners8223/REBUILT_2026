package frc.robot.subsystems.Intake;

import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;


public class IntakeConstants 
{

    public static class PositionMotor
    {
        //TODO find real info for constants!
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 0;
        public static final boolean IS_BRUSHLESS = false;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final boolean IS_INVERTED = false;
        public static final double VOLTAGE = 0;
        public static final double POSITION_TOLERANCE = 0;
        public static final double GEAR_RATIO = 0;
        public static final double ROTATIONS_TO_METERS = 0;
    }

    public static class TopMotor
    {
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 0;
        public static final boolean IS_BRUSHLESS = false;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final boolean IS_INVERTED = false; 
        public static final double VOLTAGE = 0;
        public static final double GEAR_RATIO = 0;
        public static final double ROTATIONS_TO_METERS = 0;
    }
    

    public class BottomMotor
    {
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 0;
        public static final boolean IS_BRUSHLESS = false;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final boolean IS_INVERTED = false;   
        public static final double VOLTAGE = 0;
        public static final double GEAR_RATIO = 0;
        public static final double ROTATIONS_TO_METERS = 0;
    }
    public static final double MOMENT_OF_INERTIA = 0;
    public static final double SOFT_MINIMUM = 0;
    public static final double SOFT_MAXIMUM = 0;
    
}
