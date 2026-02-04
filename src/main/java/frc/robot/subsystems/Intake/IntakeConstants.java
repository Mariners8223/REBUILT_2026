package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.VoltageUnit;
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
        public static final boolean IS_BRUSHLESS = false;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final boolean IS_INVERTED = false;
        public static final Measure<VoltageUnit> VOLTAGE = Volts.of(0);//TODO:fix
        public static final double POSITION_TOLERANCE = 0;
        public static final double GEAR_RATIO = 1;
        public static final Measure<PerUnit<AngleUnit, DistanceUnit>> ROTATIONS_PER_METERS = Rotations.per(Meters).ofNative(0);//TODO:fix
        public static final double TOP_POSITION = 0;
        public static final double MIDDLE_POSITION = 0;
        public static final double BOTTOM_POSITION = 0;
    }

    public static class TopMotor
    {
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 0;
        public static final boolean IS_BRUSHLESS = false;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final boolean IS_INVERTED = false; 
        public static final Measure<VoltageUnit> VOLTAGE = Volts.of(0);//TODO: fix
        public static final 
        public static final double GEAR_RATIO = 1;
        public static final Measure<PerUnit<AngleUnit, DistanceUnit>> ROTATIONS_PER_METERS = Rotations.per(Meters).ofNative(0);//TODO:fix
    }
    

    public class BottomMotor
    {
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 0;
        public static final boolean IS_BRUSHLESS = false;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final boolean IS_INVERTED = false;   
        public static final Measure<VoltageUnit> VOLTAGE = Volts.of(0);//TODO: fix
        public static final double GEAR_RATIO = 1;
        public static final Measure<PerUnit<AngleUnit, DistanceUnit>> ROTATIONS_PER_METERS = Rotations.per(Meters).ofNative(0);//TODO:fix
    }
    public static final double MOMENT_OF_INERTIA = 0;
    public static final double SOFT_MINIMUM = 0;
    public static final double SOFT_MAXIMUM = 0;
    
}
