package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;


public class IntakeConstants 
{

    public static class PositionMotor
    {
        //TODO find real info for constants!
        public enum IntakePosition
        {
            Bottom(Degrees.of(0)),
            Middle(Degrees.of(0)),
            Top(Degrees.of(0));

            private final Angle angle;

            IntakePosition(Angle angle)
            {
                this.angle = angle;
            }
 
            public Angle getAngle()
            {
                return this.angle;
            }

            public static IntakePosition findNearestPosition(Angle angle)
            {
                for(IntakePosition position : IntakePosition.values())
                {
                    Angle distance = position.getAngle().minus(angle);
                    if(distance.lt(IntakeConstants.PositionMotor.POSITION_TOLERANCE))
                        return position;
                }
                return null;
            }

        }
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 0;
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final boolean IS_INVERTED = false;
        //public static final Measure<VoltageUnit> VOLTAGE = Volts.of(0);//TODO: do i need to delete that? if you dont want it just do it
        public static final Angle POSITION_TOLERANCE = Degrees.of(0);;
        public static final double GEAR_RATIO = 1;
        public static final double SOFT_MINIMUM = 0;
        public static final double SOFT_MAXIMUM = 0;
        public static final Constraints PROFILE = new Constraints(0, 0);
        public static final PIDFGains PID_GAINS = new PIDFGains(
        0,
        0,
        0,
        0,
        POSITION_TOLERANCE.in(Rotation),
        0);
        public static final double X_ON_ROBOT = 0;
        public static final double Y_ON_ROBOT = 0;
        public static final double Z_OFFSET = 0;
    }

    public static class RollersMotor
    {
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 0;
        public static final boolean IS_BRUSHLESS = false;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final boolean IS_INVERTED = false; 
        //public static final Measure<VoltageUnit> VOLTAGE = Volts.of(0);//TODO: do i need to delete that? if you dont want it just do it
        public static final double GEAR_RATIO = 1;
        public static final double DUTY_CYCLE = 0; // i assume that you want a dutyCycle constat bc you have that in your io 
    }

    public static final double MOMENT_OF_INERTIA = 0;
    
    
}
