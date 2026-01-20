package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.Drive.DriveCommand;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.commands.Shooter.ShooterTest.ShooterTestCommand;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.Shooter.ShooterTest.ShooterTest;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static DriveBase driveBase;
    public static ShooterTest shooterTest;

    public static CommandPS5Controller driveController;
    public static CommandXboxController driveXboxController;


    public RobotContainer() {
        driveController = new CommandPS5Controller(1);
        driveXboxController = new CommandXboxController(0);

        driveBase = new DriveBase();
        shooterTest = new ShooterTest();

        configureDriveBindings();

        SmartDashboard.putNumber("Shooter Speed", 0.0);
        
    }


    public void configureDriveBindings(){
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(new StartEndCommand(() ->
            driveBase.setDefaultCommand(new DriveCommand(driveBase, driveXboxController)),
            driveBase::removeDefaultCommand).ignoringDisable(true));
        driveController.cross().whileTrue(new ShooterTestCommand(shooterTest));
   }
   


}
