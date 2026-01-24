
package frc.robot.subsystems.Funnel;

import frc.util.MarinersController.MarinersSparkBase.MotorType;

import frc.util.MarinersController.MarinersController.ControllerLocation;


public class FunnelingConstents {
    public static ControllerLocation controllerLocation;
    public static MotorType motorType;
    public static class LeadingMotor{
        public static final int Lead_ID = 42;
        public static final ControllerLocation ControllerLocation = controllerLocation.MOTOR;
        public static final boolean Is_Brushless = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final double GearRatio = 1;
        public static final double MOMENT_OF_INERTIA_SIM = 1;

        public static final double LeadSpeed = 0.4;
    }
    public static class CenteringHIGHMotor{
        public static final int CenterHIGH_ID = 13;
        public static final ControllerLocation CONTROLLER_LOCATION = controllerLocation.MOTOR;
        public static final boolean Is_Brushless = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_MAX; // spark flex or spark max?
                public static final double GearRatio = 1;
        public static final double MOMENT_OF_INERTIA_SIM = 1;


        public static final double CenteringHighSpeed = 0.4;
    }
    public static class CenteringLOWMotor{
        public static final int CenterLOW_ID = 17;
        public static final ControllerLocation CONTROLLER_LOCATION = controllerLocation.MOTOR;
        public static final boolean Is_Brushless = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX; // spark flex or spark max?
                public static final double GearRatio = 1;
        public static final double MOMENT_OF_INERTIA_SIM = 1;


        public static final double CenteringLowSpeed = 0.2; 
    }
}
