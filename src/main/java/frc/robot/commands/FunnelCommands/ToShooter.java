

package frc.robot.commands.FunnelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Funnel.FunnelingConstents;

public class ToShooter extends Command {
  public final Funnel funnel;
  public ToShooter(Funnel funnel) {
    this.funnel = funnel;
  }


  @Override
  public void initialize() {
    funnel.SpinCenterMotors(FunnelingConstents.CenteringLOWMotor.CenteringLowSpeed, FunnelingConstents.CenteringHIGHMotor.CenteringHighSpeed);
    funnel.SpinLeadMotor(FunnelingConstents.LeadingMotor.LeadSpeed);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    funnel.StopCenterMotors();
    funnel.StopLeadMotor();

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
