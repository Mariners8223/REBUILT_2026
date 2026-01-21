package frc.robot.subsystems.Intake;

import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersSparkBase;
import frc.robot.subsystems.Intake.IntakeConstants.PositionMotor;

public class IntakeIOReal implements IntakeIO{
    private final MarinersSparkBase positionMotor;
    private final MarinersSparkBase topMotor;
    private final MarinersSparkBase bottomMotor;

    public intakeIOReal()
    {
        this.positionMotor = configurePositionMotor();
        this.topMotor = configureTopMotor();
        this.bottomMotor = configureBottomMotor();
    }
    private MarinersSparkBase configurePositionMotor()
    {
        MarinersSparkBase motor;
        motor = new MarinersSparkBase("Position motor",IntakeConstants.PositionMotor.CONTROLLER_LOCATION, IntakeConstants.PositionMotor.MOTOR_ID,
        IntakeConstants.PositionMotor.IS_BRUSHLESS, IntakeConstants.PositionMotor.MOTOR_TYPE);
        
        motor.setMotorInverted(IntakeConstants.PositionMotor.IS_INVERTED);

        return motor;
    }
    private MarinersSparkBase configureTopMotor()
    {
        MarinersSparkBase motor;
        motor = new MarinersSparkBase("Top motor", IntakeConstants.TopMotor.CONTROLLER_LOCATION, IntakeConstants.TopMotor.MOTOR_ID,
        IntakeConstants.TopMotor.IS_BRUSHLESS, IntakeConstants.TopMotor.MOTOR_TYPE);

        motor.setMotorInverted(IntakeConstants.TopMotor.IS_INVERTED);

        return motor;
    }
    private MarinersSparkBase configureBottomMotor()
    {
        MarinersSparkBase motor;
        motor = new MarinersSparkBase("Bottom motor", IntakeConstants.BottomMotor.CONTROLLER_LOCATION, IntakeConstants.BottomMotor.MOTOR_ID,
        IntakeConstants.BottomMotor.IS_BRUSHLESS, IntakeConstants.BottomMotor.MOTOR_TYPE);

        motor.setMotorInverted(IntakeConstants.TopMotor.IS_INVERTED);

        return motor;
    }

    public void setPositionMotorVoltage(double voltage)
    {
        positionMotor.setVoltage(voltage);
    }

    public void setTopMotorVoltage(double voltage)
    {
        topMotor.setVoltage(voltage);
    }

    public void setBottomMotorVoltage(double voltage)
    {
        bottomMotor.setVoltage(voltage);
    }
}
