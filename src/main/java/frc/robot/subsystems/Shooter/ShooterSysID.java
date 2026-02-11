// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ShooterSysID {

    private final SysIdRoutine routine;
    
    public ShooterSysID(Shooter shooter){
        routine = new SysIdRoutine(
            new Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SYSID/State", state.toString())
            ),
            new Mechanism(
                (voltage) -> shooter.setVoltage(voltage),
                null,
                shooter)
            );
    }

    public Command getShooterQuasistatic(Direction direction){
        return routine.quasistatic(direction);
    }

    public Command getShooterDynamic(Direction direction){
        return routine.dynamic(direction);
    }

}
