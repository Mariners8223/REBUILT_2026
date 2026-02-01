package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.Drive.DriveCommand;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.subsystems.DriveTrain.DriveBase;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public static Constants.TrenchLocations trenchLocations = new Constants.TrenchLocations();
    public static DriveBase driveBase;
    public static Robot robot;
    public static CommandPS5Controller driveController;
    public static CommandXboxController driveXboxController;


    public RobotContainer() {
        driveController = new CommandPS5Controller(1);
        driveXboxController = new CommandXboxController(0);

        driveBase = new DriveBase();
    
        configureDriveBindings();
        
    }


    public void configureDriveBindings(){
       new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(new StartEndCommand(() ->
       driveBase.setDefaultCommand(new DriveCommand(driveBase, driveXboxController)),
       driveBase::removeDefaultCommand).ignoringDisable(true));
       driveXboxController.b().onTrue(driveBase.resetOnlyDirection());//TODO: find the options button
       driveXboxController.a().whileTrue(passTrench());
       
      
   }

   public Command passTrench(){
    Pose2d pose = driveBase.getPose();

    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        pose = FlippingUtil.flipFieldPose(driveBase.getPose());
    }
    if (pose.getY() >= 4){
        if (pose.getX() <= 4.5){
            return driveBase.findPath(trenchLocations.upRightTrench,2);
        }
        return driveBase.findPath(trenchLocations.upLeftTrench, 2);
    }
    if (pose.getX() <= 4.5){
        return driveBase.findPath(trenchLocations.downRightTrench, 2);
    }+
    return driveBase.findPath(trenchLocations.downLeftTrench, 2);
   }
   


}
