package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

import org.photonvision.PhotonPoseEstimator;

public class VisionConstants {

    public static final PhotonPoseEstimator.PoseStrategy MAIN_STRATEGY =
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    public static final PhotonPoseEstimator.PoseStrategy FALLBACK_STRATEGY =
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;

    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double maxHeightDeviation = 0.1;
    public static final double maxMultiAmbiguity = 0.2;
    public static final double maxSingleAmbiguity = 0.2;
    public static final double maxDistanceFromTargetSingle = 2.5;
    public static final double maxDistanceFromTargetMulti = 2; // If there is a season where you can see one tag real close and one far fix this.

    public enum CameraConstants{//TODO: change stdfactors!!!!!!!!!!!!!!!!!!!!!!!!!
        FRONT_CAMERA("Front Camera",
            new Transform3d(
                0.11143, 0.221, 0.574,
                new Rotation3d(0, Units.degreesToRadians(7), Units.degreesToRadians(-10))),
                0.2, 9999),

       POINTING_OUT_CAMERA("Right Camera", //this camera will be placed on the side of the shooter, pointing backwards
           new Transform3d(
               0.23554, -0.26835, 0.47246,
               new Rotation3d(Units.degreesToRadians(-15), Units.degreesToRadians(0), Units.degreesToRadians(0))),
               0.2, 9999);

        public final String cameraName;
        public final double XYstdFactor;
        public final double thetaStdFactor;

        public final Transform3d robotToCamera;

        CameraConstants(String name, Transform3d robotToCamera, double xySTD, double thetaSTD){
            this.cameraName = name;
            this.robotToCamera = robotToCamera;
            this.XYstdFactor = xySTD;
            this.thetaStdFactor = thetaSTD;
        }

        @Override
        public String toString(){
            return cameraName;
        }
    }

}
