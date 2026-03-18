// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Second;

import java.util.List;

import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.util.PathPlannerLogging;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.util.HubTracker;


public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    public static boolean isRedAlliance = false;
    HubTracker tracker = new HubTracker();

    private static final Field2d field = new Field2d();

    
    public Robot() {
        Logger.recordMetadata("ProjectName", "MyProject"); 

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
        }
        else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }

        isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
        Logger.start(); 
        new RobotContainer();
        SmartDashboard.putBoolean("IsHubActive", HubTracker.isActive(DriverStation.getAlliance().get()));
        SmartDashboard.putString("TimeLeftOnActivation", HubTracker.timeRemainingInCurrentShift().toString());
        SmartDashboard.putBoolean("InRange", RobotContainer.inRange());

        SmartDashboard.putData("Field", field);
    }

    public static void clearObjectPoseField(String name) {
        field.getObject(name).setPoses();
    }

    public static void setTrajectoryField(String name, List<Pose2d> poses) {
        field.getObject(name).setPoses(poses);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
        SmartDashboard.putNumber("Robot Velocity", RobotContainer.driveBase.getVelocity());
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());
        SmartDashboard.putNumber("PDH Voltage", ConduitApi.getInstance().getPDPVoltage());
        
        SmartDashboard.putBoolean("Is Hub Active", HubTracker.isActive(DriverStation.getAlliance().get()));
        SmartDashboard.putNumber("Time left in Shift", Math.floor(HubTracker.timeRemainingInCurrentShift().orElseGet(() -> Second.zero()).in(Second)));
        SmartDashboard.putString("Current Shift", HubTracker.getCurrentShift().orElseGet(() -> HubTracker.Shift.AUTO).toString());

        SmartDashboard.putString("Distant to Hub", String.format("%.2f", RobotContainer.distanceFromHub().in(Meter)));
        SmartDashboard.putBoolean("In Alliance Zone", RobotContainer.inAllianceZone());

        RobotContainer.pollAlerts();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}
    
    @Override
    public void autonomousInit() {
        m_autonomousCommand = RobotContainer.getAuto();

        if (m_autonomousCommand != null){
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        PathPlannerLogging.setLogTargetPoseCallback(pose -> Logger.recordOutput("Path pose", pose));
    }

    public void resetAllEncoders(){

    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        Logger.recordOutput("Distance To Hub", RobotContainer.distanceFromHub().in(Meter));
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
