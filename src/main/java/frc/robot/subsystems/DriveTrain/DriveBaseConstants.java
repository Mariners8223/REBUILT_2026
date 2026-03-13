package frc.robot.subsystems.DriveTrain;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DriveTrain.SwerveModules.DevBotConstants;
import frc.util.PIDFGains;

public class DriveBaseConstants {

    ///public static final double DISTANCE_BETWEEN_WHEELS = 0.58; // the distance between each wheel in meters
    public static final double DISTANCE_BETWEEN_WHEELS_HORIZONTAL = 0.54819; //the distance between each wheel in meters - long axis
    public static final double DISTANCE_BETWEEN_WHEELS_VERTICAL = 0.5044; //'''''' - short axis
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[]{
            new Translation2d(DISTANCE_BETWEEN_WHEELS_HORIZONTAL / 2, DISTANCE_BETWEEN_WHEELS_VERTICAL / 2),
            new Translation2d(DISTANCE_BETWEEN_WHEELS_HORIZONTAL / 2, -DISTANCE_BETWEEN_WHEELS_VERTICAL / 2),
            new Translation2d(-DISTANCE_BETWEEN_WHEELS_HORIZONTAL / 2, DISTANCE_BETWEEN_WHEELS_VERTICAL / 2),
            new Translation2d(-DISTANCE_BETWEEN_WHEELS_HORIZONTAL / 2, -DISTANCE_BETWEEN_WHEELS_VERTICAL / 2)};

    public static final double THETA_KS = 0.092706;
    public static final double THETA_KV = 0.91749;
    public static final double THETA_KA = 0.26202;

    public static final class PathPlanner {
        public static final ModuleConfig MODULE_CONFIG = DevBotConstants.MODULE_CONFIG;

        public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
                63,
                6,
                MODULE_CONFIG,
                MODULE_TRANSLATIONS);

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                2,
                6.5, //TODO find a good value for this
                9.75,
                38); //the constraints for pathPlanner

        public static final PIDFGains THETA_PID = new PIDFGains(7, 2, 0.05); //the pid gains for the PID Controller of the robot angle, units are radians per second
        public static final PIDFGains XY_PID = new PIDFGains(5, 0.2, 0.1);//the pid gains for the pid controller of the robot's velocity, units are meters per second
    }

    public static final class DrivePID{
        public static final double LOWER_SPEED_LIMIT_XY = 0.5;
        public static final double UPPER_SPEED_LIMIT_XY = 1.5;

        public static double LOWER_SPEED_LIMIT_THETA = 1;
        public static final double UPPER_SPEED_LIMIT_THETA = 4 * 1.1;

        public static final double XY_DEADBAND = 0;
        public static final double THETA_DEADBAND = 0.2;

        public static final double XY_TOLERANCE = 0.015;
        public static final double THETA_TOLERANCE = Units.degreesToRadians(2);

        public static final double THETA_IZONE = Units.degreesToRadians(10);

        public static final PIDController X_PID = new PIDController(5, 2, 0.1);
        public static final PIDController Y_PID = new PIDController(5, 2, 0.1);
        public static final PIDController THETA_PID = new PIDController(8, 4, 0);

        static{
            X_PID.setIntegratorRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            Y_PID.setIntegratorRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            THETA_PID.setIntegratorRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

            X_PID.setIZone(0.1);
            Y_PID.setIZone(0.1);
            THETA_PID.setIZone(THETA_IZONE);

            X_PID.setTolerance(XY_TOLERANCE);
            Y_PID.setTolerance(XY_TOLERANCE);
            THETA_PID.setTolerance(THETA_TOLERANCE);

            THETA_PID.enableContinuousInput(-Math.PI, Math.PI);
        }
    }

    public static final int PIGEON_ID = 2;
}
