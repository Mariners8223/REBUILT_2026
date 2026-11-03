package frc.robot.subsystems.Intake.Rollers;

import frc.util.MarinersController.MarinersTalonFX;
public class isaakrollerIOReal implements isaakroller{
    private final MarinersTalonFX FirstMotor;
    private final MarinersSparkBase FollowMotor;
        public RollersIOReal(){
       FirstMotor = new MarinersTalonFX("Intake/First Rollers Motor", RollerConstants.CONTROLLER_LOCATION,
                                    RollerConstants.MOTOR_ID);

        FirstMotor.setCurrentLimits(RollerConstants.CURRENT_LIMIT, RollerConstants.CURRENT_THRESHOLD);
        FirstMotor.setMotorInverted(RollerConstants.IS_INVERTED);
        FirstMotor.getMeasurements().setGearRatio(RollerConstants.GEAR_RATIO);

        SecMotor = new MarinersTalonFX("Intake/Sec Rollers Motor", RollerConstants.CONTROLLER_LOCATION,
                                    RollerConstants.SEC_MOTOR_ID);

        SecMotor.setCurrentLimits(RollerConstants.CURRENT_LIMIT, RollerConstants.CURRENT_THRESHOLD);
        SecMotor.setMotorInverted(RollerConstants.IS_SEC_INVERTED);
        SecMotor.getMeasurements().setGearRatio(RollerConstants.GEAR_RATIO);

    }
    public void setDutyCycle(double dutyCycle){
        FirstMotor.setDutyCycle(dutyCycle);
        SecMotor.setDutyCycle(dutyCycle);
    }

    public void update(RollersInputs inputs){}

    

}