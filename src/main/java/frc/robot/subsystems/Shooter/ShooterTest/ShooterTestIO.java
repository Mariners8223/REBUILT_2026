// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.ShooterTest;

import org.littletonrobotics.junction.AutoLog;



/** Add your docs here. */
public interface ShooterTestIO {
    @AutoLog
    class ShooterInputsTest{
        
    }
    void setDutyCycle(double speedPercentage);//between 1 and -1
    void stop ();//dutyCycle = 0
    void update(ShooterInputsTest inputs);
    
} 
