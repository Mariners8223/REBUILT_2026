// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootDistance extends Command {
  Shooter shooter;
  Supplier<Distance> distanceSupplier;

  Timer boostTimer;

  /** Creates a new Shoot. */
  public ShootDistance(Shooter shooter, Supplier<Distance> distanceSupplier) {
    this.shooter = shooter;
    this.distanceSupplier = distanceSupplier;
    boostTimer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  public AngularVelocity requiredSpeed(){
    return ShooterConstants.Calculations.requiredAngularVelocity(distanceSupplier.get());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AngularVelocity requiredSpeed = requiredSpeed();
    shooter.setVelocity(requiredSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.getShooterAcceleration().unaryMinus().gte(ShooterConstants.FALL_ACCELERATION)){
      boostTimer.restart();
      shooter.setDutyCycle(1);
    }

    if (shooter.getShooterVelocity().gte(requiredSpeed())){
      boostTimer.stop();
    }

    if (!boostTimer.isRunning()) shooter.setVelocity(requiredSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVoltage(Volts.zero());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
