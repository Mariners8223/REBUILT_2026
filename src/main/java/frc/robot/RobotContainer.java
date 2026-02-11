// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.Drive.DriveCommand;

import frc.robot.subsystems.DriveTrain.DriveBase;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Intake.Intake;

public class RobotContainer {
    public static CommandPS5Controller driveController;
    public static CommandXboxController driveXboxController;

    public static DriveBase driveBase;
    public static Funnel funnel;
    public static Intake intake;


    public RobotContainer() {
        driveController = new CommandPS5Controller(1);
        driveXboxController = new CommandXboxController(0);

        driveBase = new DriveBase();
        funnel = new Funnel();
        intake = new Intake();
        
        configureDriveBindings();
    }


    public void configureDriveBindings(){
          new Trigger(RobotState::isTeleop).and(RobotState::isEnabled).whileTrue(new StartEndCommand(() ->
              driveBase.setDefaultCommand(new DriveCommand(driveBase, driveXboxController)),
              driveBase::removeDefaultCommand).ignoringDisable(true));        
   }
}
