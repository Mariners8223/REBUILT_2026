// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IncreasingShootSpeed extends Command {
    Shooter shooter;
    int timer;
    AngularVelocity startingSpeed;
    double speed;

    /** Creates a new IncreasingShootSpeed. */
    public IncreasingShootSpeed(Shooter shooter, AngularVelocity startingSpeed) {
        this.shooter = shooter;

        this.startingSpeed = startingSpeed;

        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.timer = 0;
        shooter.setShooterVelocity(startingSpeed);
        shooter.setKickerVoltage(Volt.of(3));
        this.speed = RPM.of(SmartDashboard.getNumber("Shooter Velocity", 2500)).in(RotationsPerSecond);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.speed += SmartDashboard.getNumber("Grow", 1);
        shooter.setShooterVelocity(RotationsPerSecond.of(speed));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelocity(startingSpeed);
        shooter.setKickerVoltage(Volt.zero());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    return false;
    }
    }
