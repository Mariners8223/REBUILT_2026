// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Kicker extends SubsystemBase {
  KickerIO io;
  KickerInputsAutoLogged inputs = new KickerInputsAutoLogged();

  /** Creates a new Kicker. */
  public Kicker() {
    io = Robot.isReal() ? new KickerIOReal() : new KickerIOSim();
  }

  public void setMotors(double dutyCycle){
    io.setDutyCycle(dutyCycle);
  }

  public void stopMotors(){
    io.setDutyCycle(0);
  }

  public Command setKickerCommand(double dutyCycle){
    return this.startEnd(
      () -> setMotors(dutyCycle),
      () -> stopMotors());
  }

  public Command setKickerByDistance(Supplier<Distance> distanceSupplier){
    return this.runEnd(
      () -> this.setMotors(KickerConstants.getRPM(distanceSupplier.get())),
      () -> this.stopMotors()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.update(inputs);
  }
}
