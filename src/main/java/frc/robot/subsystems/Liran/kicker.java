package frc.robot.subsystems.Liran;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class kicker extends SubsystemBase{

    public final KickerIO io;

    public kicker(){
        io = new kickerIOReal();
    }

    public void wheelturn(double a,double b){
        io.setspeedrollers(a);
        io.setspeedfunneling(b);
    }
    public void stopmotors(){
        io.setspeedfunneling(0);
        io.setspeedrollers(0);
    }
}
