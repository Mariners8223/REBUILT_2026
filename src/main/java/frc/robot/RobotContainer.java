package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.Drive.DriveCommand;

import edu.wpi.first.wpilibj.RobotState;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;

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

    public static MarinersSparkBase shooterMotor1;
    public static MarinersSparkBase shooterMotor2;
    public static MarinersTalonFX kickerMotor1;
    public static MarinersTalonFX kickerMotor2;


    public RobotContainer() {
        driveController = new CommandPS5Controller(1);
        driveXboxController = new CommandXboxController(0);

        driveBase = new DriveBase();

        configureDriveBindings();


        shooterMotor1 = new MarinersSparkBase("Shooter Motor 1", ControllerLocation.MOTOR, 0, true, MotorType.SPARK_MAX);
        shooterMotor2 = new MarinersSparkBase("Shooter Motor 2", ControllerLocation.MOTOR, 0, true, MotorType.SPARK_MAX);
        shooterMotor2.setMotorAsFollower(shooterMotor1, true);

        kickerMotor1 = new MarinersTalonFX("Kicker Motor 1", ControllerLocation.MOTOR, 0);
        kickerMotor2 = new MarinersTalonFX("Kicker Motor 2", ControllerLocation.MOTOR, 0);
        kickerMotor2.setMotorAsFollower(kickerMotor1, false);
        
    }


    public void configureDriveBindings(){
        new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(new StartEndCommand(() ->
            driveBase.setDefaultCommand(new DriveCommand(driveBase, driveXboxController)),
            driveBase::removeDefaultCommand).ignoringDisable(true));
   }
   


}
