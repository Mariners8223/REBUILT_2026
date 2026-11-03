package frc.robot.subsystems.Intake.Rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
  RollersIO io;
  RollersInputsAutoLogged inputs = new RollersInputsAutoLogged();

  public Rollers(){
    io = new RollersIOReal();
  }
  public void setDutyCycle(double dutyCycle){
    io.setDutyCycle(dutyCycle);
    }
  public void noMotor(){
    io.setDutyCycle(0);
  }
  @Override
  public void periodic() {
    io.update(inputs);
  

}

