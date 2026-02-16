package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.Climb.RunHookToHeight;
import frc.robot.commands.Drive.MinorAdjust;
import frc.robot.commands.Drive.MinorAdjust.AdjustmentDirection;
import frc.robot.subsystems.Climb.Climb;
import frc.robot.subsystems.Climb.ClimbConstants.Heights;
import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Climb.ClimbConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    public static CommandPS5Controller driveController;
    public static DriveBase driveBase;
    public static Climb climb;

    public RobotContainer() {
        driveController = new CommandPS5Controller(0);
        driveBase = new DriveBase();
        climb = new Climb();

        configureClimbTestBindings();
    }


    public void configureClimbTestBindings(){
        driveController.povRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.RIGHT));
        driveController.povLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.LEFT));
        driveController.povUp().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FORWARD));
        driveController.povDown().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACKWARDS));
        driveController.povUpLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_LEFT));
        driveController.povUpRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.FRONT_RIGHT));
        driveController.povDownLeft().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_LEFT));
        driveController.povDownRight().whileTrue(new MinorAdjust(driveBase, AdjustmentDirection.BACK_RIGHT));

        
        driveController.triangle().onTrue(climb.toPositionCommand(Heights.EXTENDED));

        driveController.R2().whileTrue(new RunHookToHeight(climb, Heights.RESET, 1));
        driveController.L2().whileTrue(new RunHookToHeight(climb, Heights.IN_AIR_AUTO, 1));
        
        driveController.square().whileTrue(new RunHookToHeight(climb, Heights.EXTENDED, ClimbConstants.GETTING_DOWN_DUTY_CYCLE));

        driveController.options().onTrue(new InstantCommand(() -> climb.resetPosition()));
   }
}
