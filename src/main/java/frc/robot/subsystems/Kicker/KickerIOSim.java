// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Kicker;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.util.MarinersController.MarinersSimMotor;

/** Add your docs here. */
public class KickerIOSim implements KickerIO {
    MarinersSimMotor kickerMotors;

    public KickerIOSim(){
        kickerMotors = new MarinersSimMotor(
            "Kicker Motors",
            DCMotor.getNeoVortex(2),
            KickerConstants.MOTOR_CONSTANTS.GEAR_RATIO,
            1
        );
    }

    public void setDutyCycle(double dutyCycle){
        kickerMotors.setDutyCycle(dutyCycle);
    }

    public void update(KickerInputs inputs){
        inputs.dutyCycle = 0;
        inputs.velocity = 0;
    }
}
