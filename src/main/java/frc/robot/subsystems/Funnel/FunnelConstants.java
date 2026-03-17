
package frc.robot.subsystems.Funnel;

import frc.util.MarinersController.MarinersSparkBase.MotorType;
import frc.util.MarinersController.MarinersController.ControllerLocation;


public class FunnelConstants {
    public static ControllerLocation controllerLocation;
    public static MotorType motorType;
    public static class funnelMotor{
        public static final int Lead_ID = 49;
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final double GEAR_RATIO = 1;
        public static final double MOMENT_OF_INERTIA_SIM = 1;
        public static final int LEAD_MOTOR_CURRENT_LIMIT = 50;
        public static final int LEAD_MOTOR_CURRENT_THRESHOLD = 60;

        public static final double LEAD_SPEED = 0.3;
        public static final double FUNNEL_SHOOTING_SPEED = 0.4;
        public static final boolean IS_INVERTED = false;
    }

    public static class CenteringMotor{
        public static final int CENTERING_ID = 33;
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final double GEAR_RATIO = 3;
        public static final double MOMENT_OF_INERTIA_SIM = 1;
        public static final int CENTER_MOTOR_CURRENT_LIMIT = 100;
        public static final int CENTER_MOTOR_CURRENT_THRESHOLD = 120;

        public static final double CenteringHighSpeed = 0.5;
        public static final double centeringShootingSpeed = 0.8;
        public static final double CENTERING_EJECT_SPEED = -0.3;
        public static final boolean IS_INVERTED = true;
    }
}
