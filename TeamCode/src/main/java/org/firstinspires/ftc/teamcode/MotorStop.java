package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class MotorStop extends CommandBase {
    private MotorSubsystem motor;

    public MotorStop(MotorSubsystem subsystem){
        motor = subsystem;
        addRequirements(motor);
    }
    public void execute(){
        motor.stop();
    }
}
