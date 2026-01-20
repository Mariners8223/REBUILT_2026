// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.ShooterTest;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.Shooter.ShooterTest.ShooterTestConstants.Motor1;



/** Add your docs here. */
public interface ShooterTestIO {
    @AutoLog
    public static class ShooterInputsTest{

        public double speed;
        public double sliderValue;
        }
    
    void setDutyCycle(double speed);//between 1 and -1
    void stop ();//dutyCycle = 0
    double getSpeed();
    void Update(ShooterInputsTest inputs);
    
} 
