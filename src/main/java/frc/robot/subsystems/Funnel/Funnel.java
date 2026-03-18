
package frc.robot.subsystems.Funnel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
  public static Alert funnelStall = new Alert("Stall", "Funnel in stall", AlertType.kWarning);
  public static Alert centeringStall = new Alert("Stall", "Centerring in stall", AlertType.kWarning);

  public final FunnelIO io;

  public Funnel() {
    io = new FunnelIOReal();
  }

  public void setFunnelDutyCycle(double dutyCycle){
    io.SetDutyCycleLead(dutyCycle);
  }

  public void stopFunnel(){
    io.SetDutyCycleLead(0);
  }

  public void setCenteringDutyCycle(double CenterMotorSpeedHIGH){
    io.SetDutyCycleCenter(CenterMotorSpeedHIGH);
  }
  public void stopCentering(){
    io.SetDutyCycleCenter(0);
  }

  public void stopAllMotors(){
    stopFunnel();
    stopCentering();
  }

  public boolean funnelInStall(){
    return (RobotState.isEnabled()) && (io.getFunnelSetpoint() != 0 && Math.abs(io.getFunnelVelocity()) < 1);
  }

  public boolean centeringInStall(){
    return (RobotState.isEnabled()) && (io.getCenterringSetpoint() != 0 && Math.abs(io.getCenterringVelocity()) < 1);
  }

  public Command funnelingCommand(){
    return this.startEnd(
      () -> setFunnelDutyCycle(FunnelConstants.funnelMotor.LEAD_SPEED),
      () -> stopFunnel()
    );
  }

  public Command centeringCommand(){
    return this.startEnd(
      () -> setCenteringDutyCycle(FunnelConstants.CenteringMotor.CenteringHighSpeed),
      () -> stopCentering()
      );
  }

  public Command toShooterCommand(){
    return this.startEnd(
      () -> {
        setFunnelDutyCycle(FunnelConstants.funnelMotor.FUNNEL_SHOOTING_SPEED);
        setCenteringDutyCycle(FunnelConstants.CenteringMotor.centeringShootingSpeed);
      },
      () -> stopAllMotors()
    );
  }


  @Override
  public void periodic() {
    Logger.recordOutput("Funnel/Command", (getCurrentCommand() != null ? getCurrentCommand().toString() : "None"));

    Logger.recordOutput("Funnel/Funnel in Stall", funnelInStall());
    Logger.recordOutput("Funnel/Centering in Stall", centeringInStall());

    funnelStall.set(funnelInStall());
    centeringStall.set(centeringInStall());
  }
}
