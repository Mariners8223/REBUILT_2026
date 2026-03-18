// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Funnel.FunnelConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Kicker.KickerConstants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.util.ContinuousConditionalCommand;

public class Feeder extends SubsystemBase {
  private final Intake intake;
  private final Funnel funnel;
  private final Kicker kicker;
  private final Shooter shooter;

  /** Creates a new Feeder. */
  public Feeder(Intake intake, Funnel funnel, Kicker kicker, Shooter shooter) {
    this.intake = intake;
    this.funnel = funnel;
    this.kicker = kicker;
    this.shooter = shooter;
  }

  public ArrayList<Subsystem> requiredSubsystems(double intakeSpeed, double funnellingSpeed, double centeringSpeed, double kickerSpeed, double shooterSpeed){
    ArrayList<Subsystem> requiredSubsystems = new ArrayList<>();

    if (intakeSpeed != 0) requiredSubsystems.add(intake);
    if (funnellingSpeed != 0 || centeringSpeed != 0) requiredSubsystems.add(funnel);
    if (kickerSpeed != 0) requiredSubsystems.add(kicker);
    if (shooterSpeed != 0) requiredSubsystems.add(shooter);

    return requiredSubsystems;
  }

  public Runnable runMotors(double intakeSpeed, double funnellingSpeed, double centeringSpeed, double kickerSpeed, double shooterSpeed){
    return () -> {
        intake.setRollersDutyCycle(intakeSpeed);
        funnel.setFunnelDutyCycle(funnellingSpeed);
        funnel.setCenteringDutyCycle(centeringSpeed);
        kicker.setDutyCycle(kickerSpeed);
        shooter.setDutyCycle(shooterSpeed);
      };
  }

  public Runnable stopMotors(){
      return () -> {
        intake.setRollersDutyCycle(0);
        funnel.stopAllMotors();
        kicker.stopMotors();
        shooter.stopShooter();
      };
  }

  public Command setSpeeds(double intakeSpeed, double funnellingSpeed, double centeringSpeed, double kickerSpeed, double shooterSpeed){
    Subsystem[] requirementsArray = requiredSubsystems(intakeSpeed, funnellingSpeed, centeringSpeed, kickerSpeed, shooterSpeed).toArray(new Subsystem[0]);

    return Commands.startEnd(
      runMotors(intakeSpeed, funnellingSpeed, centeringSpeed, kickerSpeed, shooterSpeed),
      stopMotors(),
      requirementsArray
    );
  }

  public boolean inStall(){
    return funnel.funnelInStall() || funnel.centeringInStall() || kicker.leadInStall() || kicker.followInStall();
  }

  public Command intakeCommand(){
    return setSpeeds(IntakeConstants.RollersMotor.DUTY_CYCLE, FunnelConstants.funnelMotor.LEAD_SPEED, 0, 0, 0);
  }
  public Command ejectCommand(){
    return setSpeeds(0, 0, FunnelConstants.CenteringMotor.CENTERING_EJECT_SPEED, KickerConstants.KICKER_EJECT_SPEED, 0);
  }

  public Command toShooterCommand(Distance distanceToHub){
    return setSpeeds(1, FunnelConstants.funnelMotor.FUNNEL_SHOOTING_SPEED, FunnelConstants.CenteringMotor.CenteringHighSpeed, KickerConstants.getDutyCycle(distanceToHub), 0);
  }
  public Command toPassCommand(){
    return setSpeeds(1, FunnelConstants.funnelMotor.FUNNEL_SHOOTING_SPEED, FunnelConstants.CenteringMotor.CenteringHighSpeed, 1, 0);
  }

  public Command smartFeedingShootCommand(Supplier<Distance> distanceSupplier){
    return Commands.repeatingSequence(
      new ContinuousConditionalCommand(
        ejectCommand().withTimeout(0.4),
        toShooterCommand(distanceSupplier.get()).withTimeout(0.4),
        this::inStall)
    );
  }
  public Command smartFeedingPassCommand(){
    return Commands.repeatingSequence(
      new ContinuousConditionalCommand(
        ejectCommand().withTimeout(0.4),
        toPassCommand(),
        this::inStall)
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
