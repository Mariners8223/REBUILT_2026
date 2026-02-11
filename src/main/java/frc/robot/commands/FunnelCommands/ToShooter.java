

package frc.robot.commands.FunnelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Funnel.FunnelConstants;

public class ToShooter extends Command {
  public final Funnel funnel;
  public ToShooter(Funnel funnel) {
    this.funnel = funnel;
    addRequirements(funnel);
  }


  @Override
  public void initialize() {
    funnel.SpinCenterMotors(FunnelConstants.CenteringMotor.CenteringHighSpeed);
    funnel.SpinLeadMotor(FunnelConstants.LeadingMotor.LeadSpeed);
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
