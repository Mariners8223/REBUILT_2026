package frc.robot.subsystems.Climb;

import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.PIDFGains;

public class ClimbConstants 
{
    public enum Heights{
        RESET(0),
        EXTENDED(0.2),
        IN_AIR(0.1);

        private final double height;

        Heights(double height){
            this.height = height;
        }

        public double getHeight(){
            return this.height;
        }
    }
    
    public static final int MOTOR_ID = 25;
    public static final double GEAR_RATIO = 45;
    public static final double ROTATIONS_TO_METERS = 1/0.1324;
    public static final double CLIMB_POWER = 0.3;
    public static final double SOFT_MINIMUM = 0; 
    public static final double SOFT_MAXIMUM = 0.8;
    public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
    public static final double CLIMB_TOLERANCE = 0.005;

    public static final boolean IS_INVERTED = true;
    public static final double MOMENT_OF_INERTIA = 1;
    public static final double START_POSITION = 0;
    public static final double X_ON_ROBOT = 0;
    public static final double Y_ON_ROBOT = 0;
    public static final double Z_OFFSET = 0;

    public static final PIDFGains PID = new PIDFGains(1000, 40, 0, 0.2);
}
