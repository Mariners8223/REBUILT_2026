// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
  public static Alert rollersStall = new Alert("Rollers in stall", AlertType.kWarning);

  RollersIO io;
  RollersInputsAutoLogged inputs = new RollersInputsAutoLogged();

  /** Creates a new Rollers. */
  public Rollers() {
    io = new RollersIOReal();
  }

  public void setDutyCycle(double dutyCycle)
  {
    io.setDutyCycle(dutyCycle);
  }
  public void stopMotor(){
    io.setDutyCycle(0);
  }

  public boolean inStall(){
    return (RobotState.isEnabled()) && (io.getSetpoint() != 0 && Math.abs(io.getVelocity()) < 1);
  }

  @Override
  public void periodic() {
    io.update(inputs);
    Logger.processInputs(getName(), inputs);

    Logger.recordOutput("Intake/Rollers in Stall", inStall());
    // This method will be called once per scheduler run
  }
}
