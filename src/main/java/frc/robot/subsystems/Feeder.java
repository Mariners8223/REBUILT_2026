// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Kicker.Kicker;
import frc.robot.subsystems.Shooter.Shooter;

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
    return this.startEnd(
      runMotors(intakeSpeed, funnellingSpeed, centeringSpeed, kickerSpeed, shooterSpeed),
      stopMotors()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
