package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.Drive.DriveCommand;
import frc.robot.commands.Shooter.ShootVelocity;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterSysID;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static DriveBase driveBase;

    public static CommandPS5Controller driveController;
    public static CommandXboxController driveXboxController;

    public static Shooter shooter;
    public static ShooterSysID shooterSysID;


    public RobotContainer() {
        driveController = new CommandPS5Controller(1);
        driveXboxController = new CommandXboxController(0);

        // driveBase = new DriveBase();

        shooter = new Shooter();
        shooterSysID = new ShooterSysID(shooter);

        // configureDriveBindings();
        configureSysIDConfig();
        
        // driveController.cross().whileTrue(
        //     new StartEndCommand(
        //         () -> shooter.setKickerVoltage(Volt.of(3)),
        //         () -> shooter.setKickerVoltage(Volt.of(0)),
        //         shooter
        //     )
        // );
        // driveController.cross().whileTrue(new IncreasingShootSpeed(shooter, RPM.of(SmartDashboard.getNumber("Shooter Velocity", 0))))
        // .whileFalse(new InstantCommand(() -> shooter.setShooterVelocity(RPM.of(SmartDashboard.getNumber("Shooter Velocity", 0)))));

        driveController.cross().whileTrue(new ShootVelocity(shooter, RobotContainer::dashboardVelocity));
        driveController.circle().whileTrue(new StartEndCommand(
            () -> shooter.setVelocity(RPM.of(SmartDashboard.getNumber("Shooter Velocity", 0))),
            () -> shooter.setVoltage(Volt.zero()), 
            shooter)
        );
        // driveController.triangle().whileTrue(new StartEndCommand(
        //     () -> shooter.setKickerVoltage(Volt.of(SmartDashboard.getNumber("Kicker Voltage", 0))),
        //     () -> shooter.setKickerVoltage(Volt.zero()), 
        //     null));
        
    }

    public static AngularVelocity dashboardVelocity(){
        return RPM.of(SmartDashboard.getNumber("Shooter Velocity", 0));
    }

    public Distance distanceFromHub(){
        return Meters.of(driveBase.getPose().getTranslation().getDistance(Constants.FieldConstants.HUB_POSITION));
    }


    public void configureDriveBindings(){
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(new StartEndCommand(() ->
            driveBase.setDefaultCommand(new DriveCommand(driveBase, driveXboxController)),
            driveBase::removeDefaultCommand).ignoringDisable(true));
   }

   public void configureSysIDConfig(){


}

}