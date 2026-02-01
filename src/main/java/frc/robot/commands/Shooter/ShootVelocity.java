// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
public class ShootVelocity extends Command {
  Shooter shooter;
  Supplier<AngularVelocity> velocitySupplier;

  AngularVelocity filteredRPM;
  AngularVelocity filteredRPMLast;

  AngularVelocity RPMLast;

  Timer boostTimer;

  /** Creates a new Shoot. */
  public ShootVelocity(Shooter shooter, Supplier<AngularVelocity> distanceSupplier) {
    this.shooter = shooter;
    this.velocitySupplier = distanceSupplier;
    boostTimer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AngularVelocity requiredSpeed = velocitySupplier.get();
    filteredRPM = requiredSpeed.copy();
    filteredRPMLast = filteredRPM;
    RPMLast = shooter.getShooterVelocity();
    Logger.recordOutput("Filtered RPM", filteredRPM);

    shooter.setShooterVelocity(requiredSpeed);
    shooter.setKickerVoltage(ShooterConstants.KICKER_ACTIVE_VOLTAGE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterVelocity(velocitySupplier.get());

    filteredRPMLast = filteredRPM.copy();
    AngularVelocity velocityDifference = shooter.getShooterVelocity().minus(filteredRPM);
    filteredRPM = filteredRPM.plus(velocityDifference.times(ShooterConstants.LOW_PASS_FILTER_ALPHA));
    Logger.recordOutput("Filtered RPM", filteredRPM);

    AngularVelocity filteredVelocityDifference = filteredRPM.minus(filteredRPMLast);
    AngularVelocity currentVelocityDifference = shooter.getShooterVelocity().minus(RPMLast);
    // if (filteredVelocityDifference.lte(ShooterConstants.SHOOTING_VELOCITY_FALL.unaryMinus())){
    if (currentVelocityDifference.lte(ShooterConstants.SHOOTING_VELOCITY_FALL.unaryMinus())){
      double boost = MathUtil.clamp(Math.abs(currentVelocityDifference.in(RotationsPerSecond)) * ShooterConstants.FEED_FORWARD_SHOOTER_BOOST,
                                0.0,
                                ShooterConstants.MAX_FEED_FORWARD_BOOST);

      shooter.boostFeedForward(boost);
      boostTimer.restart();
    }

    if (boostTimer.hasElapsed(ShooterConstants.FEED_FORWARD_BOOST_TIME) || ShooterConstants.Calculations.epsilonEquals(shooter.getShooterVelocity(), velocitySupplier.get(), RotationsPerSecond.of(1))){
      shooter.resetFeedForward();
      boostTimer.stop();
    }

    RPMLast = shooter.getShooterVelocity().copy();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooterVoltage(Volts.zero());
    shooter.setKickerVoltage(Volts.zero());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
