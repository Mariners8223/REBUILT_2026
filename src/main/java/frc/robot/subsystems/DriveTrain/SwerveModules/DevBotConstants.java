package frc.robot.subsystems.DriveTrain.SwerveModules;

import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.util.PIDFGains;

public enum DevBotConstants {
    FRONT_LEFT(3, 37, 8,
            false, false, -0.359619,
            new PIDFGains(50, 0, 0), //Drive motor PID
            new PIDFGains(25, 20, 0, 0, 0.001, 0.05), //Steer motor PID
            0.12069, 2.4564, 0.50368, 0.40371, 1.7422, 0.25834),

    FRONT_RIGHT(4, 40, 14,
    false, false, 0.081299,
            new PIDFGains(50, 0, 0), //Drive motor PID
            new PIDFGains(25, 20, 0, 0,  0.001, 0.05), //Steer motor PID
            0.045019, 2.4174, 0.76547, 0.3149, 1.7429, 0.10638),

    BACK_LEFT(12, 5, 50,
    false, false, 0.003174,
            new PIDFGains(50, 0, 0), //Drive motor PID
            new PIDFGains(25, 20, 0, 0, 0.001, 0.05), //Steer motor PID
            0.061361, 2.5147, 0.54424, 0.35616, 1.6894, 0.1793),

    BACK_RIGHT(9, 44, 11,
    false, false, -0.08252,
            new PIDFGains(50, 0, 0), //Drive motor PID
            new PIDFGains(25, 20, 0, 0, 0.001, 0.05), //Steer motor PID
            0.16489, 2.4765, 0.35734, 0.37585, 1.7989, 0.12709);

    public static final double DRIVE_GEAR_RATIO = 6.75;
    public static final double STEER_GEAR_RATIO = 12.8;
    public static final double WHEEL_RADIUS_METERS = 0.0508;
    public static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;
    public static final double MAX_WHEEL_LINEAR_VELOCITY = 4;
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;//112
    public static final int DRIVE_MOTOR_CURRENT_THRESHOLD = 20;//70-s

    //acceleration and jerk constraints for the drive motor
    public static final TrapezoidProfile.Constraints DRIVE_CONSTRAINTS = new TrapezoidProfile.Constraints(18, 100);

    //velocity and acceleration constraints for the steer motor
    public static final TrapezoidProfile.Constraints STEER_CONSTRAINTS = new TrapezoidProfile.Constraints(20, 25);

    public static final ModuleConfig MODULE_CONFIG = new ModuleConfig(
        WHEEL_RADIUS_METERS,
        MAX_WHEEL_LINEAR_VELOCITY,
        0.9,
        DCMotor.getFalcon500(2),
        DRIVE_GEAR_RATIO,
        DRIVE_MOTOR_CURRENT_LIMIT,
        1);

    /**
     * the motor id for the drive motor
     */
    public final int DRIVE_MOTOR_ID;

    /**
     * the motor id for the steer motor
     */
    public final int STEER_MOTOR_ID;

    /**
     * the motor id for the absolute encoder
     */
    public final int ABSOLUTE_ENCODER_ID;

    /**
     * if the drive motor is inverted (meaning positive is counter-clockwise)
     */
    public final boolean DRIVE_INVERTED;

    /**
     * if the steer motor is inverted (meaning positive is counter-clockwise)
     */
    public final boolean STEER_INVERTED;

    /**
     * the offset between the zero of the magnet of the encoder and the zero of the module (in rotations)
     */
    public final double ABSOLUTE_ZERO_OFFSET;

    /**
     * the PIDF gains for the drive motor
     */
    public final PIDFGains DRIVE_MOTOR_PID;

    /**
     * the PIDF gains for the steer motor
     */
    public final PIDFGains STEER_MOTOR_PID;

    /**
     * the kS for the drive motor
     * the voltage needed to overcome static friction
     * units are volts
     */
    public final double DRIVE_KS;

    /**
     * the kV for the drive motor
     * units are volts per radian per second (V/rad/s)
     */
    public final double DRIVE_KV;

    /**
     * the kA for the drive motor
     * units are volts per radian per second squared (V/rad/s^2)
     */
    public final double DRIVE_KA;

    /**
     * the kS for the steer motor
     * the voltage needed to overcome static friction
     * units are volts
     */
    public final double STEER_KS;

    /**
     * the kV for the steer motor
     * units are volts per radian per second (V/rad/s)
     */
    public final double STEER_KV;

    /**
     * the kA for the steer motor
     * units are volts per radian per second squared (V/rad/s^2)
     */
    public final double STEER_KA;

    DevBotConstants(int DRIVE_MOTOR_ID, int STEER_MOTOR_ID, int ABSOLUTE_ENCODER_ID, boolean driveInverted,
                     boolean steerInverted, double absZeroOffset, PIDFGains driveMotorPID, PIDFGains steerMotorPID,
                     double DRIVE_KS, double DRIVE_KV, double DRIVE_KA, double STEER_KS, double STEER_KV, double STEER_KA) {
        this.DRIVE_INVERTED = driveInverted;
        this.STEER_INVERTED = steerInverted;
        this.ABSOLUTE_ZERO_OFFSET = absZeroOffset;
        this.DRIVE_MOTOR_PID = driveMotorPID;
        this.STEER_MOTOR_PID = steerMotorPID;

        this.DRIVE_MOTOR_ID = DRIVE_MOTOR_ID;
        this.STEER_MOTOR_ID = STEER_MOTOR_ID;
        this.ABSOLUTE_ENCODER_ID = ABSOLUTE_ENCODER_ID;

        this.DRIVE_KS = DRIVE_KS;
        this.DRIVE_KV = DRIVE_KV;
        this.DRIVE_KA = DRIVE_KA;

        this.STEER_KS = STEER_KS;
        this.STEER_KV = STEER_KV;
        this.STEER_KA = STEER_KA;
    }
}
