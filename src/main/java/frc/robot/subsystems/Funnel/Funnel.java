
package frc.robot.subsystems.Funnel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
  public final FunnelIO io;
  // public final FunnelInputsAutoLogged inputs = new FunnelInputsAutoLogged(); 
  public Funnel() {
    io = new FunnelIOReal();
  }

  
  public void SpinLeadMotor(double LeadMotorSpeed){
    io.SetDutyCycleLead(LeadMotorSpeed);
  }

  public void StopLeadMotor(){
    io.SetDutyCycleLead(0);
  }
  
  public void SpinCenterMotors(double CenterMotorSpeedLOW , double CenterMotorSpeedHIGH){
    io.setDutyCycleCenterLow(CenterMotorSpeedLOW);
    io.SetDutyCycleCenterHigh(CenterMotorSpeedHIGH);
  }
  public void StopCenterMotors(){
    io.setDutyCycleCenterLow(0);
    io.SetDutyCycleCenterHigh(0);
  }


  @Override
  public void periodic() {
    // io.update(inputs);
  }
}
