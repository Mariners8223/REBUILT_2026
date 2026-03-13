
package frc.robot.subsystems.Funnel;

import frc.util.MarinersController.MarinersSparkBase.MotorType;
import frc.util.MarinersController.MarinersController.ControllerLocation;


public class FunnelConstants {
    public static ControllerLocation controllerLocation;
    public static MotorType motorType;
    public static class LeadingMotor{
        public static final int Lead_ID = 49;
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final boolean Is_Brushless = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final double GearRatio = 1;
        public static final double MOMENT_OF_INERTIA_SIM = 1;
        public static final int LEAD_MOTOR_CURRENT_LIMIT = 50;
        public static final int LEAD_MOTOR_CURRENT_THRESHOLD = 60;

        public static final double LeadSpeed = 0.3;
        public static final boolean IS_INVERTED = false;
    }

    public static class CenteringMotor{
        public static final int CenterHIGH_ID = 33;
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final double GearRatio = 3;
        public static final double MOMENT_OF_INERTIA_SIM = 1;
        public static final int CENTER_MOTOR_CURRENT_LIMIT = 120;
        public static final int CENTER_MOTOR_CURRENT_THRESHOLD = 120;

        public static final double CenteringHighSpeed = 0.5;
        public static final double CENTERING_EJECT_SPEED = -0.3;
        public static final boolean IS_INVERTED = true;
    }
}
