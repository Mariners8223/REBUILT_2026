
package frc.robot.subsystems.Funnel;

import frc.util.MarinersController.MarinersSparkBase.MotorType;
import static frc.util.MarinersController.MarinersController.ControllerLocation;


public class FunnelingConstents {
    public static ControllerLocation controllerLocation;
    public static MotorType motorType;
    public static class LeadingMotor{
        public static final int Lead_ID = 12;
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final boolean Is_Brushless = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final double GearRatio = 0.6;
        public static final double MOMENT_OF_INERTIA_SIM = 1;

        public static final double LeadSpeed = 0.3;
        public static final boolean IS_INVERTED = true;
    }
    public static class CenteringMotor{
        public static final int CenterHIGH_ID = 33;
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final double GearRatio = 3;
        public static final double MOMENT_OF_INERTIA_SIM = 1;


        public static final double CenteringHighSpeed = 0.4;
        public static final boolean IS_INVERTED = false;
    }
    /*public static class CenteringLOWMotor{
        public static final int CenterLOW_ID = 17;
        public static final ControllerLocation CONTROLLER_LOCATION = controllerLocation.MOTOR;
        public static final boolean Is_Brushless = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX; // spark flex or spark max?
                public static final double GearRatio = 1;
        public static final double MOMENT_OF_INERTIA_SIM = 1;


        public static final double CenteringLowSpeed = 0.3; 
    }*/
}
