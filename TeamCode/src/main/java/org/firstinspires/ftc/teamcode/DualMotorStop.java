package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class DualMotorStop extends CommandBase {
    private DualMotorSubsystem motor;

    public DualMotorStop(DualMotorSubsystem subsystem){
        motor = subsystem;
        addRequirements(motor);
    }
    public void execute(){
        motor.stop();
    }
}
