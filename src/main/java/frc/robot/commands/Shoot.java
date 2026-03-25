// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;


import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  Shooter shooter;
  Supplier<AngularVelocity> velocitySupplier;

  boolean boosting = false;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, Supplier<AngularVelocity> velocitySupplier) {
    this.shooter = shooter;
    this.velocitySupplier = velocitySupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    setName("Shoot Velocity");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AngularVelocity requiredSpeed = velocitySupplier.get();
    shooter.setVelocity(requiredSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.getShooterAcceleration().unaryMinus().gte(ShooterConstants.FALL_ACCELERATION)){
      boosting = true;
      shooter.setDutyCycle(1);
    }

    if (shooter.getShooterVelocity().gte(velocitySupplier.get())){
      boosting = false;
    }

    if (!boosting) shooter.setVelocity(velocitySupplier.get());
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


  public static AngularVelocity requiredSpeed(Distance distance){
    return ShooterConstants.requiredAngularVelocity(distance);
  }

  public static Command ShootDistance(Shooter shooter, Supplier<Distance> distanceSupplier){
    return new Shoot(shooter, () -> requiredSpeed(distanceSupplier.get())).withName("Shoot Distance");
  }
}
