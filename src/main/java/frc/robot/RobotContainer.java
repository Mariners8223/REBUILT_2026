package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Drive.DriveCommand;

import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotState;
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
        
    }

    public Distance distanceFromHub(){
        return driveBase.getPose().minus(Constants.FieldConstants.HUB_POSITION);
    }


    public void configureDriveBindings(){
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(new StartEndCommand(() ->
            driveBase.setDefaultCommand(new DriveCommand(driveBase, driveXboxController)),
            driveBase::removeDefaultCommand).ignoringDisable(true));
   }

   public void configureSysIDConfig(){
        driveXboxController.y().whileTrue(shooterSysID.getShooterQuasistatic(Direction.kForward));
        driveXboxController.a().whileTrue(shooterSysID.getShooterQuasistatic(Direction.kReverse));

        driveXboxController.b().whileTrue(shooterSysID.getShooterDynamic(Direction.kForward));
        driveXboxController.x().whileTrue(shooterSysID.getShooterDynamic(Direction.kReverse));

        driveXboxController.povUp().whileTrue(
            new StartEndCommand(
                () -> shooter.setShooterVoltage(Volt.of(4)),
                () -> shooter.setShooterVoltage(Volt.zero()),
                shooter)
        );
   }
   


}
