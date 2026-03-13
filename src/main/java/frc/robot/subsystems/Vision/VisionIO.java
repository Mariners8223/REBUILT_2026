package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;


public interface VisionIO {
    VisionFrame[] emptyFrame =
            {new VisionFrame(false, -1, -1, new Pose3d(), 1,
                    EstimationType.SINGLE_TARGET, -1, -1)};

    void update(VisionInputsAutoLogged inputs);


     record VisionFrame(
            boolean hasTarget,
            double timeStamp,
            double latency,
            Pose3d robotPose,
            double poseAmbiguity,
            EstimationType estimationType,
            double averageTargetDistance,
            double tagCount){}

    enum EstimationType{
        SINGLE_TARGET(VisionConstants.maxSingleAmbiguity, VisionConstants.maxDistanceFromTargetSingle),
        MULTIPLE_TARGETS(VisionConstants.maxMultiAmbiguity, VisionConstants.maxDistanceFromTargetMulti);

        EstimationType(double maxAmbiguity, double maxDistance) {
            this.maxAmbiguity = maxAmbiguity;
            this.maxDistance = maxDistance;
        }

        private final double maxAmbiguity;
        private final double maxDistance;

        public double getMaxAmbiguity() {
            return maxAmbiguity;
        }
        public double getMaxDistance() {
            return maxDistance;
        }
    }

    @AutoLog
    class VisionInputs{
        boolean isConnected;
        VisionFrame[] visionFrames = new VisionFrame[0];
        int[] targetIDs = new int[0];
    }

}
