package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants;
import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;


public class IntakeConstants
{

    public static class PositionMotor
    {
        public enum IntakeStates
        {
            Closed(Degrees.of(0)),
            Middle(Degrees.of(-45)), //TODO: Test bump heights
            Open(Degrees.of(-90)),
            Reset(Degrees.of(-90)); //TODO: Change before competition

            private final Angle angle;

            IntakeStates(Angle angle)
            {
                this.angle = angle;
            }

            public Angle getAngle()
            {
                return this.angle;
            }

            public static IntakeStates findNearestPosition(Angle angle)
            {
                for(IntakeStates position : IntakeStates.values())
                {
                    if (Constants.CALCULATIONS.epsilonEquals(
                        angle,
                        position.getAngle(),
                        IntakeConstants.PositionMotor.POSITION_TOLERANCE)
                    ) return position;
                }
                return null;
            }

        }
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 56;
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final boolean IS_INVERTED = true;
        public static final Angle POSITION_TOLERANCE = Degrees.of(2);
        public static final double GEAR_RATIO = 60;
        public static final double SOFT_MINIMUM = IntakeStates.Open.getAngle().in(Rotations);
        public static final double SOFT_MAXIMUM = IntakeStates.Closed.getAngle().in(Rotations);
        public static final PIDFGains PID_GAINS = new PIDFGains(
        800,
        50,
        0,
        0.1);
        public static final TrapezoidProfile PROFILE = new TrapezoidProfile(
            new Constraints(2, 0.8)
        );
        public static final double STALL_CURRENT = 40;
    }

    public static class RollersMotor
    {
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 23;
        public static final boolean IS_BRUSHLESS = false;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final boolean IS_INVERTED = false;
        public static final double GEAR_RATIO = 5;
        public static final double DUTY_CYCLE = 1.0;
    }

    public static final double MOMENT_OF_INERTIA = 0;


    public static final double X_ON_ROBOT = 0;
    public static final double Y_ON_ROBOT = 0;
    public static final double Z_OFFSET = 0;


    public static final Time BUMP_WAIT_TIME = Seconds.of(0.5);

}
