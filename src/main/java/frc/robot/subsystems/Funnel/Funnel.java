
package frc.robot.subsystems.Funnel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase {
  public static Alert funnelStall = new Alert("Funnel in stall", AlertType.kWarning);
  public static Alert centeringStall = new Alert("Centerring in stall", AlertType.kWarning);

  public final FunnelIO io;

  public Funnel() {
    io = new FunnelIOReal();
  }

  public void setFunnelDutyCycle(double dutyCycle){
    io.setDutyCycleFunnel(dutyCycle);
  }

  public void stopFunnel(){
    io.setDutyCycleFunnel(0);
  }

  public void setCenteringDutyCycle(double centeringDutyCycle){
    io.setDutyCycleCenter(centeringDutyCycle);
  }
  public void stopCentering(){
    io.setDutyCycleCenter(0);
  }

  public void stopAllMotors(){
    stopFunnel();
    stopCentering();
  }

  public boolean funnelInStall(){
    return (RobotState.isEnabled()) && (io.getFunnelSetpoint() != 0 && Math.abs(io.getFunnelVelocity()) < 1);
  }

  public boolean centeringInStall(){
    return (RobotState.isEnabled()) && (io.getCenteringSetpoint() != 0 && Math.abs(io.getCenterringVelocity()) < 1);
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
