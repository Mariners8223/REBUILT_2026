package frc.robot.commands.SuperShoot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.commands.Shooter.ShootVelocity;
import frc.robot.commands.Drive.TurnToSetAngle;


public class SuperShootCommand extends ParallelCommandGroup {

    private final DriveBase driveBase;

    private Pose2d hubLocation;
    private Pose2d currentTarget;

    public SuperShootCommand(DriveBase driveBase, Shooter shooter, Kicker kicker, Funnel funnel, Pose2d hubLocation) {
        this.driveBase = driveBase;

        currentTarget = hubLocation;
        this.hubLocation = hubLocation;

        addCommands(
            Commands.run(() -> currentTarget = getVirtualTarget()),
            new ShootVelocity(shooter, () -> {
                double distance = currentTarget.getTranslation().getDistance(driveBase.getPose().getTranslation());
                return Units.RPM.of(getRPM(distance));
            }),
            new TurnToSetAngle(driveBase, () -> {
                Rotation2d rotation = currentTarget.getTranslation().minus(driveBase.getPose().getTranslation()).getAngle();
                return Units.Radians.of(rotation.getRadians());
            }),
            kicker.setKickerCommand(0.6),
            funnel.toShooterCommand()
        );
    }

    private double getFlightTime(double distance) {
        return distance * 8223; // dummy — replace with tree table lookup
    }

    private double getRPM(double distance) {
        return distance / 8223; //dummy 
    }

    private Pose2d getVirtualTarget() {
        return getVirtualTarget(currentTarget, 0);
    }

    private Pose2d getVirtualTarget(Pose2d virtualTarget, int depth) {
        if (depth >= 5) return virtualTarget;

        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
            driveBase.getChassisSpeeds(), driveBase.getRotation2d()
        );

        double distance = driveBase.getPose().getTranslation().getDistance(virtualTarget.getTranslation());

        double flightTime = getFlightTime(distance);

        Translation2d virtualOffset = new Translation2d(
            -fieldRelative.vxMetersPerSecond * flightTime,
            -fieldRelative.vyMetersPerSecond * flightTime
        );

        Pose2d newVirtualTarget = new Pose2d(
            hubLocation.getTranslation().plus(virtualOffset),
            hubLocation.getRotation()
        );

        if (newVirtualTarget.getTranslation()
                .getDistance(virtualTarget.getTranslation()) < 0.1) {
            return newVirtualTarget;
        }

        return getVirtualTarget(newVirtualTarget, depth + 1);
    }
}