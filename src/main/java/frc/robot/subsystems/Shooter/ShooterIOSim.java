// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.util.MarinersController.MarinersSimMotor;
import frc.util.MarinersController.MarinersController.ControlMode;

/** Add your docs here. */
public class ShooterIOSim implements ShooterIO{
    FlywheelSim flywheel;

    MarinersSimMotor kickerMotors;


    public ShooterIOSim(){
        flywheel = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(4), 
                ShooterConstants.SHOOTER_MOMENT_OF_INERTIA.in(KilogramSquareMeters), 
                ShooterConstants.MOTOR_CONSTANTS.GEAR_REDUCTION
            ),
            DCMotor.getKrakenX60(2),
            0.0001, 0.0001
        );
    }

    public double getVelocity(){
        return flywheel.getAngularVelocityRPM();
    }
    public void setVelocity(double targetVelocity){
        flywheel.setInput(targetVelocity);
    }
    public void setVoltage(double voltage){
        flywheel.setInput(voltage);
    }
    public void setDutyCycle(double targetDutyCycle){
        flywheel.setInputVoltage(targetDutyCycle / flywheel.getGearbox().nominalVoltageVolts);
    }
    public void feedForwardBoost(double boost){
        return;
    }
    public void resetFeedForward(){
        return;
    }

    public void update(ShooterInputs inputs){
        inputs.shooterVelocity = RPM.of(getVelocity());

        LinearVelocity shooterLinearVelocity = Meter.per(Minute).of(
            getVelocity() * ShooterConstants.SHOOTER_WHEEL_CIRCUMFERENCE.in(Meter)
        );
        inputs.shooterLinearVelocity = shooterLinearVelocity;

        inputs.pose = ShooterConstants.POSITION;
    }
}
