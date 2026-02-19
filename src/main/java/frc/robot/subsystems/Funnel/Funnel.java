
package frc.robot.subsystems.Funnel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
  public final FunnelIO io;

  public Funnel() {
    io = new FunnelIOReal();
  }

  public void SpinLeadMotor(double LeadMotorSpeed){
    io.SetDutyCycleLead(LeadMotorSpeed);
  }

  public void StopLeadMotor(){
    io.SetDutyCycleLead(0);
  }

  public void SpinCenterMotors(double CenterMotorSpeedHIGH){
    io.SetDutyCycleCenter(CenterMotorSpeedHIGH);
  }
  public void StopCenterMotors(){
    io.SetDutyCycleCenter(0);
  }

  public void stopAllMotors(){
    StopLeadMotor();
    StopCenterMotors();
  }

  public Command funnelingCommand(){
    return this.startEnd(
      () -> SpinLeadMotor(FunnelConstants.LeadingMotor.LeadSpeed),
      () -> StopLeadMotor()
    );
  }

  public Command onlyCenterCommand(){
    return this.startEnd(
      () -> SpinCenterMotors(FunnelConstants.CenteringMotor.CenteringHighSpeed),
      () -> StopCenterMotors()
      );
  }

  public Command toShooterCommand(){
    return this.startEnd(
      () -> {
        SpinLeadMotor(FunnelConstants.LeadingMotor.LeadSpeed);
        SpinCenterMotors(FunnelConstants.CenteringMotor.CenteringHighSpeed);
      },
      () -> stopAllMotors()
    );
  }


  @Override
  public void periodic() {
    Logger.recordOutput("Funnel/Command", (getCurrentCommand() != null ? getCurrentCommand().toString() : "None"));
  }
}
