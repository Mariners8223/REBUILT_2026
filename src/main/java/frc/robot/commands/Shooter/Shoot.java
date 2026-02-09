// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  Shooter shooter;
  Supplier<Distance> distanceSupplier;

  AngularVelocity filteredRPM;
  AngularVelocity filteredRPMLast;

  Timer boostTimer;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, Supplier<Distance> distanceSupplier) {
    this.shooter = shooter;
    this.distanceSupplier = distanceSupplier;
    boostTimer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AngularVelocity requiredSpeed = ShooterConstants.Calculations.requiredAngularVelocity(distanceSupplier.get());
    filteredRPM = requiredSpeed.copy();
    filteredRPMLast = filteredRPM;
    Logger.recordOutput("Filtered RPM", filteredRPM);

    shooter.setVelocity(requiredSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocityByDistance(distanceSupplier.get());

    filteredRPMLast = filteredRPM.copy();
    AngularVelocity velocityDifference = shooter.getShooterVelocity().minus(filteredRPM);
    filteredRPM = filteredRPM.plus(velocityDifference.times(ShooterConstants.LOW_PASS_FILTER_ALPHA));
    Logger.recordOutput("Filtered RPM", filteredRPM);

    AngularVelocity filteredVelocityDifference = filteredRPM.minus(filteredRPMLast);
    if (filteredVelocityDifference.lte(ShooterConstants.SHOOTING_VELOCITY_FALL.unaryMinus())){
      double boost = MathUtil.clamp(Math.abs(filteredVelocityDifference.in(RotationsPerSecond)) * ShooterConstants.FEED_FORWARD_SHOOTER_BOOST,
                                0.0,
                                ShooterConstants.MAX_FEED_FORWARD_BOOST);

      shooter.boostFeedForward(boost);
      boostTimer.restart();
    }

    if (boostTimer.hasElapsed(ShooterConstants.FEED_FORWARD_BOOST_TIME)){
      shooter.resetFeedForward();
      boostTimer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
