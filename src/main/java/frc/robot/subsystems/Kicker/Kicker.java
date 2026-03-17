// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Kicker extends SubsystemBase {
  public static Alert kickerLeadStall = new Alert("Stall", "Lead Kicker in stall", AlertType.kWarning); // TODO: Front or back kicker
  public static Alert kickerFollowStall = new Alert("Stall", "Following Kicker in stall", AlertType.kWarning); // TODO: Front or back kicker


  KickerIO io;
  KickerInputsAutoLogged inputs = new KickerInputsAutoLogged();

  /** Creates a new Kicker. */
  public Kicker() {
    io = Robot.isReal() ? new KickerIOReal() : new KickerIOSim();
  }

  public void setDutyCycle(double dutyCycle){
    io.setDutyCycle(dutyCycle);
  }

  public void stopMotors(){
    io.setDutyCycle(0);
  }

  public boolean leadInStall(){
    return (io.getLeadSetpoint() != 0 && Math.abs(io.getLeadVelocity()) < 1);
  }
  public boolean followInStall(){
    return (io.getFollowSetpoint() != 0 && Math.abs(io.getFollowVelocity()) < 1);
  }

  public Command setKickerCommand(double dutyCycle){
    return this.startEnd(
      () -> setDutyCycle(dutyCycle),
      () -> stopMotors());
  }

  public Command setKickerByDistance(Supplier<Distance> distanceSupplier){
    return this.runEnd(
      () -> this.setDutyCycle(KickerConstants.getRPM(distanceSupplier.get())),
      () -> this.stopMotors()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.update(inputs);

    kickerLeadStall.set(leadInStall());
    kickerFollowStall.set(followInStall());
  }
}
