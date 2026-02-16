// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootVelocity extends Command {
  Shooter shooter;
  Supplier<AngularVelocity> velocitySupplier;
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
    RPMLast = shooter.getShooterVelocity();

    System.out.println("Fall");
    Logger.recordOutput("Shooter/Fall", (shooter.getShooterAcceleration().unaryMinus().gte(ShooterConstants.FALL_ACCELERATION)));
    shooter.setVelocity(requiredSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocity(velocitySupplier.get());
    Logger.recordOutput("Shooter/Fall", (shooter.getShooterAcceleration().unaryMinus().gte(ShooterConstants.FALL_ACCELERATION)));

    // AngularVelocity velocityDifference = shooter.getShooterVelocity().minus(RPMLast);
    // if (velocityDifference.lte(ShooterConstants.SHOOTING_VELOCITY_FALL.unaryMinus())){
    if (shooter.getShooterAcceleration().unaryMinus().gte(ShooterConstants.FALL_ACCELERATION)){
      double boost = MathUtil.clamp(shooter.getShooterAcceleration().unaryMinus().in(RotationsPerSecondPerSecond) * ShooterConstants.FEED_FORWARD_SHOOTER_BOOST,
                                0.0,
                                ShooterConstants.MAX_FEED_FORWARD_BOOST);

      shooter.boostFeedForward(boost);
      boostTimer.restart();
    }

    if (boostTimer.hasElapsed(ShooterConstants.FEED_FORWARD_BOOST_TIME)
        || ShooterConstants.Calculations.epsilonEquals(shooter.getShooterVelocity(), velocitySupplier.get(), RotationsPerSecond.of(1))){
      shooter.resetFeedForward();
      boostTimer.stop();
    }

    RPMLast = shooter.getShooterVelocity().copy();
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
