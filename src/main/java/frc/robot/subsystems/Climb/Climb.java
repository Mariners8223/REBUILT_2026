package frc.robot.subsystems.Climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();

    /**
     * Creates a new Climb.
     */
    public Climb() {
        io = Robot.isReal() ?  new ClimbIOSim();
        io.resetPosition();

        io.setBrakeMode(false);

        new Trigger(RobotState::isEnabled).whileTrue(new StartEndCommand(
            () -> io.setBrakeMode(true),
            () -> io.setBrakeMode(false)).ignoringDisable(true));
    }

    public void setMotorPower(double power) {
        io.setPower(power);
    }
    public void setServoAngle(double angle){
        io.setServoAngle(angle);
    }

    public boolean isAtLimit() {
        return (io.getPosition() <= ClimbConstants.SOFT_MINIMUM);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.Update(inputs);
        Logger.processInputs(getName(), inputs);

        double climbPrecnet = (inputs.height / ClimbConstants.SOFT_MINIMUM) * 100;

        SmartDashboard.putNumber("climb precent", climbPrecnet);

        String currentCommandName = getCurrentCommand() == null ? "Null" : getCurrentCommand().getName();
        Logger.recordOutput("Climb/Current Command", currentCommandName);
    }
}