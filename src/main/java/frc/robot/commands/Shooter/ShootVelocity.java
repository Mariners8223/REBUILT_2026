// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
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

  double boost;
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
    Logger.recordOutput("Shooter/Boost", boost);
    shooter.setVelocity(requiredSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("Shooter/Fall", (shooter.getShooterAcceleration().unaryMinus().gte(ShooterConstants.FALL_ACCELERATION)));

    if (shooter.getShooterAcceleration().unaryMinus().gte(ShooterConstants.FALL_ACCELERATION)){
      boost += MathUtil.clamp(shooter.getShooterAcceleration().unaryMinus().in(RotationsPerSecondPerSecond) * ShooterConstants.FEED_FORWARD_SHOOTER_BOOST,
                              0.0,
                              ShooterConstants.MAX_FEED_FORWARD_BOOST);

      // shooter.boostFeedForward(boost);
      boostTimer.restart();
      shooter.setDutyCycle(1);
    }

    // if (boostTimer.hasElapsed(ShooterConstants.FEED_FORWARD_BOOST_TIME)
    //     || shooter.getShooterVelocity().gte(velocitySupplier.get())){
    if (shooter.getShooterVelocity().gte(velocitySupplier.get())){
      // shooter.resetFeedForward();
      boostTimer.stop();
      boost = 0;
    }

    if (!boostTimer.isRunning()) shooter.setVelocity(velocitySupplier.get());
    RPMLast = shooter.getShooterVelocity().copy();

    Logger.recordOutput("Shooter/Boost", boost);
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
