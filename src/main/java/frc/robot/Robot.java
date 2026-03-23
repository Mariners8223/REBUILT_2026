// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.util.FlippingUtil;
import com.pathplanner.lib.util.PathPlannerLogging;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.util.Elastic;


public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;
    public static boolean isRedAlliance = false;

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
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        new RobotContainer();

        PathPlannerLogging.setLogTargetPoseCallback(pose -> Logger.recordOutput("Path Pose", pose));

        SmartDashboard.putData("Field", field);
    }

    public static void clearObjectPoseField(String name) {
        field.getObject(name).setPoses();
    }

    public static void setTrajectoryField(String name, List<Pose2d> poses) {
        field.getObject(name).setPoses(poses);
    }

    public static void setRobotPoseOnField(Pose2d pose) {
        if(Robot.isRedAlliance){
            pose = FlippingUtil.flipFieldPose(pose);
        }
        field.setRobotPose(pose);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();

        RobotContainer.pollAlerts();
        RobotContainer.updateElastic();
        RobotContainer.updateLogging();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        Elastic.selectTab("Autonomous");

        m_autonomousCommand = RobotContainer.getAuto();

        if (m_autonomousCommand != null){
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        Elastic.selectTab("Teleop");

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
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
