package frc.robot.subsystems.DriveTrain;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveTrain.SwerveModules.DevBotConstants;
import frc.util.PIDFGains;

public class DriveBaseConstants {
        
    ///public static final double DISTANCE_BETWEEN_WHEELS = 0.58; // the distance between each wheel in meters
    public static final double DISTANCE_BETWEEN_WHEELS_HORIZONTAL = 0.5044; //the distance between each wheel in meters - long axis
    public static final double DISTANCE_BETWEEN_WHEELS_VERTICAL = 0.54222; //'''''' - short axis
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[]{
            new Translation2d(DISTANCE_BETWEEN_WHEELS_HORIZONTAL / 2, DISTANCE_BETWEEN_WHEELS_VERTICAL / 2),
            new Translation2d(DISTANCE_BETWEEN_WHEELS_HORIZONTAL / 2, -DISTANCE_BETWEEN_WHEELS_VERTICAL / 2),
            new Translation2d(-DISTANCE_BETWEEN_WHEELS_HORIZONTAL / 2, DISTANCE_BETWEEN_WHEELS_VERTICAL / 2),
            new Translation2d(-DISTANCE_BETWEEN_WHEELS_HORIZONTAL / 2, -DISTANCE_BETWEEN_WHEELS_VERTICAL / 2)};

    public static final double THETA_KS = 0.11014;
    public static final double THETA_KV = 0.87023;
    public static final double THETA_KA = 0.35338;

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

    public static final int PIGEON_ID = 2;
}
