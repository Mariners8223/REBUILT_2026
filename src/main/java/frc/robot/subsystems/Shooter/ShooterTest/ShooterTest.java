// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.ShooterTest;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterTest.ShooterTestIO.ShooterInputsTest;

public class ShooterTest extends SubsystemBase {
  private final ShooterTestIO io;
  private final ShooterInputsTestAutoLogged inputs = new ShooterInputsTestAutoLogged();
  //private final ShooterInputsTest inputs = new ShooterInputsTest();
  /** Creates a new ShooterTest. */
  public ShooterTest() {
    this.io = new ShooterTestIOReal();
    

  }  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.Update(inputs);
    Logger.processInputs("ShooterTest", inputs);

    setDutyCycle(inputs.sliderValue);
  }
  
  public void setDutyCycle(double speed)
  {
    io.setDutyCycle(speed);
  }

  public void stop()
  {
    io.setDutyCycle(0);
  }
}
