// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.Shooter.ShooterTest;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Shooter.ShooterTest.ShooterTestConstants.Motor2;
import frc.robot.subsystems.Shooter.ShooterTest.ShooterTestIO.ShooterInputsTest;
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControlMode;


/** Add your docs here. */
public class ShooterTestIOReal {


    private final MarinersTalonFX Motor1;
    private final MarinersTalonFX Moter2;


    public ShooterTestIOReal(){
        Motor1 = configureMotor1();
        Motor2 = configureMotor2();
    }


    private MarinersTalonFX configureMotor1() {
        MarinersTalonFX motor = new MarinersTalonFX("MOTOR1", ShooterTestConstants.Motor1.CONTROLLER_LOCATION,
                                                    ShooterTestConstants.Motor1.MOTOR_ID, ShooterTestConstants.Motor1.PID_GAINS,
                                                    ShooterTestConstants.Motor1.GEAR_RATION);
        motor.setMotorInverted(ShooterTestConstants.Motor1.IS_INVERTED);
        motor.setMotorIdleMode(true);
    }


    private MarinersTalonFX configureMotor2() {
        MarinersTalonFX motor = new MarinersTalonFX("MOTOR2", ShooterTestConstants.Motor2.CONTROLLER_LOCATION,
                                                    ShooterTestConstants.Motor2.MOTOR_ID, ShooterTestConstants.Motor2.PID_GAINS,
                                                    ShooterTestConstants.Motor2.GEAR_RATION);
        motor.setMotorInverted(ShooterTestConstants.Motor2.IS_INVERTED);
        motor.setMotorIdleMode(true);
    }


    public void setDutyCycle(double speed) {
        Motor1.setReference(speed, ControlMode.DutyCycle);
        Motor2.setReference(speed, ControlMode.DutyCycle);
    }
    public void stop() {
        setDutyCycle(0);
    }
    public double getSpeed() {
        return Motor1.getVelocity();
    }

    public void Update(ShooterInputsTest inputs) {
        inputs.speed = Motor1.getVelocity();
        inputs.sliderValue = SmartDashboard.getNumber("Shooter Slider", 0.0);
    }
}



