// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
    private final ShooterIO io; 
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    /** Creates a new Shooter. */
    public Shooter() {
        io = Robot.isReal() ? new ShooterIOReal() : new ShooterIOSim();
    }

    public AngularVelocity getShooterVelocity(){ 
        return inputs.shooterVelocity; 
    }
    public LinearVelocity getShooterLinearVelocity(){
        return inputs.shooterLinearVelocity;
    }

    public void stopShooter(){
        io.setShooterDutyCycle(0);
    }
    public void setShooterVelocity(AngularVelocity targetVelocity){ 
        io.setShooterVelocity(targetVelocity.in(RotationsPerSecond)); 
    }
    public void setShooterLinearVelocity(LinearVelocity targetVelocity){
        double targetAngularVelocity = targetVelocity.in(Meters.per(Minute)) / ShooterConstants.SHOOTER_WHEEL_CIRCUMFERENCE.in(Meters);
        io.setShooterVelocity(targetAngularVelocity);
    }
    public void setShooterVoltage(Voltage voltage){
        io.setShooterVoltage(voltage.in(Volts));
    }
    public void boostFeedForward(double boost){
        io.shooterFeedForwardBoost(boost);
    }
    public void resetFeedForward(){
        io.resetShooterFeedForward();
    }

    
    public AngularVelocity getKickerVelocity(){ 
        return inputs.kickerVelocity;
    }
    public LinearVelocity getKickerLinearVelocity(){
        return inputs.kickerLinearVelocity;
    }

    public void setKickerVelocity(AngularVelocity targetVelocity){ 
        io.setKickerVelocity(targetVelocity.in(RotationsPerSecond)); 
    }
    public void setKickerLinearVelocity(LinearVelocity targetVelocity){
        double targetAngularVelocity = targetVelocity.in(Meters.per(Minute)) / ShooterConstants.KICKER_WHEEL_CIRCUMFERENCE.in(Meters);
        io.setShooterVelocity(targetAngularVelocity);
    }
    public void setKickerVoltage(Voltage voltage){
        io.setKickerVoltage(voltage.in(Volts));
    }


    public void setShooterVelocityByDistance(Distance distance){
        setShooterVelocity(ShooterConstants.Calculations.requiredAngularVelocity(distance));
    }
    public boolean isAtRequiredVelocity(Distance distance){
        return ShooterConstants.Calculations.epsilonEquals(
            getShooterVelocity(), 
            ShooterConstants.Calculations.requiredAngularVelocity(distance),
            ShooterConstants.SHOOTER_ANGULAR_VELOCITY_TOLERANCE
        );
    }


    @Override
    public void periodic() {
        io.update(inputs);
    }
}
