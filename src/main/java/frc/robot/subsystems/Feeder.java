// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
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
import frc.util.ContinuousConditionalCommand;

public class Feeder extends SubsystemBase {
  private final Intake intake;
  private final Funnel funnel;
  private final Kicker kicker;

  /** Creates a new Feeder. */
  public Feeder(Intake intake, Funnel funnel, Kicker kicker) {
    this.intake = intake;
    this.funnel = funnel;
    this.kicker = kicker;
  }

  public ArrayList<Subsystem> requiredSubsystems(double intakeSpeed, double funnellingSpeed, double centeringSpeed, double kickerSpeed){
    ArrayList<Subsystem> requiredSubsystems = new ArrayList<>();

    if (intakeSpeed != 0) requiredSubsystems.add(intake);
    if (funnellingSpeed != 0 || centeringSpeed != 0) requiredSubsystems.add(funnel);
    if (kickerSpeed != 0) requiredSubsystems.add(kicker);

    return requiredSubsystems;
  }

  public Runnable runMotors(double intakeSpeed, double funnellingSpeed, double centeringSpeed, double kickerSpeed){
    return () -> {
        intake.setRollersDutyCycle(intakeSpeed);
        funnel.setFunnelDutyCycle(funnellingSpeed);
        funnel.setCenteringDutyCycle(centeringSpeed);
        kicker.setDutyCycle(kickerSpeed);
      };
  }

  public Runnable stopMotors(){
      return () -> {
        intake.setRollersDutyCycle(0);
        funnel.stopAllMotors();
        kicker.stopMotors();
      };
  }

  public Command setSpeeds(double intakeSpeed, double funnellingSpeed, double centeringSpeed, double kickerSpeed){
    Subsystem[] requirementsArray = requiredSubsystems(intakeSpeed, funnellingSpeed, centeringSpeed, kickerSpeed).toArray(new Subsystem[0]);

    return Commands.startEnd(
      runMotors(intakeSpeed, funnellingSpeed, centeringSpeed, kickerSpeed),
      stopMotors(),
      requirementsArray
    );
  }

  public boolean inStall(){
    return funnel.funnelInStall() || funnel.centeringInStall() || kicker.leadInStall() || kicker.followInStall();
  }

  public Command intakeCommand(){
    return setSpeeds(IntakeConstants.RollersMotor.DUTY_CYCLE, FunnelConstants.funnelMotor.FUNNEL_INTAKE_SPEED, 0, 0).
        withName("Feeder Intake");
  }
  public Command ejectCommand(){
    return setSpeeds(0, 0, FunnelConstants.CenteringMotor.CENTERING_EJECT_SPEED, KickerConstants.KICKER_EJECT_SPEED).
        withName("Feeder Eject");
  }
  public Command passEjectCommand(){
    return setSpeeds(-0.8, -0.6, -0.2, 0).
        withName("Feeder Eject Pass");
  }

  public Command toShooterCommand(Distance distanceToHub){
    return setSpeeds(0.8, FunnelConstants.funnelMotor.FUNNEL_SHOOTING_SPEED, FunnelConstants.CenteringMotor.CENTERING_SHOOTING_SPEED, KickerConstants.getDutyCycle(distanceToHub)).
        withName("Feeder to Shoot");
  }
  public Command toPassCommand(){
    return setSpeeds(0.8, FunnelConstants.funnelMotor.FUNNEL_SHOOTING_SPEED, FunnelConstants.CenteringMotor.CENTERING_SHOOTING_SPEED, 1).
        withName("Feeder to Pass");
  }

  public Command smartFeedingShootCommand(Supplier<Distance> distanceSupplier, BooleanSupplier withAutoEjecting){
    return Commands.repeatingSequence(
      new ContinuousConditionalCommand(
        ejectCommand().withTimeout(0.4),
        toShooterCommand(distanceSupplier.get()).withTimeout(0.4),
        () -> (inStall() && withAutoEjecting.getAsBoolean()))
    ).withName("Smart Feeding to Shoot");
  }
  public Command smartFeedingPassCommand(BooleanSupplier withAutoEjecting){
    return Commands.repeatingSequence(
      new ContinuousConditionalCommand(
        ejectCommand().withTimeout(0.4),
        toPassCommand(),
        () -> (inStall() && withAutoEjecting.getAsBoolean()))
    ).withName("Smart Feeding to Pass");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
