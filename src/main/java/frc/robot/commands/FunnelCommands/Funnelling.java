package frc.robot.commands.FunnelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Funnel.Funnel;
import frc.robot.subsystems.Funnel.FunnelingConstents;

public class Funnelling extends Command {
  public final Funnel funnel;
  public Funnelling(Funnel funnel) {
    this.funnel = funnel;
  }

  @Override
  public void initialize() {
    funnel.SpinLeadMotor(FunnelingConstents.LeadingMotor.LeadSpeed);
  }

  @Override
  public void execute() {

  }


  @Override
  public void end(boolean interrupted) {
    funnel.StopLeadMotor();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
