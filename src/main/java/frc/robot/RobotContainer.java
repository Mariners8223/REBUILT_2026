package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.Climb.RunHookToHeight;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbConstants.Heights;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public static CommandPS5Controller driveController;
    public static Climb climb;

    public RobotContainer() {
        driveController = new CommandPS5Controller(0);
        climb = new Climb();

        configureDriveBindings();
    }


    public void configureDriveBindings(){
        driveController.cross().onTrue(climb.toPositionCommand(Heights.RESET));
        driveController.triangle().onTrue(climb.toPositionCommand(Heights.EXTENDED));
        driveController.square().onTrue(climb.toPositionCommand(Heights.IN_AIR_TELEOP));

        driveController.R1().whileTrue(
            new RunHookToHeight(climb, Heights.IN_AIR_TELEOP, 1)
        );

        driveController.povUp().whileTrue(
            Commands.startEnd(
                () -> climb.setMotorPower(0.1),
                () -> climb.setMotorPower(0),
                climb)
        );
        driveController.povDown().whileTrue(
            Commands.startEnd(
                () -> climb.setMotorPower(-0.1),
                () -> climb.setMotorPower(0),
                climb)
        );

        driveController.options().onTrue(new InstantCommand(() -> climb.resetPosition()));
   }
}
