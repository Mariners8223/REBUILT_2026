package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;


public class IntakeConstants 
{

    public static class PositionMotor
    {
        //TODO find real info for constants!
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 0;
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final boolean IS_INVERTED = false;
        //public static final Measure<VoltageUnit> VOLTAGE = Volts.of(0);//TODO: do i need to delete that? if you dont want it just do it
        public static final Angle POSITION_TOLERANCE = Degrees.of(0);;
        public static final double GEAR_RATIO = 1;
        public static final Angle TOP_POSITION = Degrees.of(0);
        public static final Angle MIDDLE_POSITION = Degrees.of(0);
        public static final Angle BOTTOM_POSITION = Degrees.of(0);
        public static final double SOFT_MINIMUM = 0;
        public static final double SOFT_MAXIMUM = 0;
    }

    public static class TopMotor
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
    

    public class BottomMotor
    {
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 0;
        public static final boolean IS_BRUSHLESS = false;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final boolean IS_INVERTED = true;   
        //public static final Measure<VoltageUnit> VOLTAGE = Volts.of(0);//TODO: do i need to delete that? if you dont want it just do it
        public static final double GEAR_RATIO = 1;
        public static final double DUTY_CYCLE = 0; // i assume that you want a dutyCycle constat bc you have that in your io
    }

    public static final double MOMENT_OF_INERTIA = 0;
    public static final double X_ON_ROBOT = 0;
    public static final double Y_ON_ROBOT = 0;
    public static final double Z_OFFSET = 0;
    
}
