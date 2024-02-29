package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class MotorRaise extends CommandBase {
    private MotorSubsystem motor;

    public MotorRaise(MotorSubsystem subsystem){
        motor = subsystem;
        addRequirements(motor);
    }
    public void execute(){
        motor.raise();
    }
    public void end(){
        motor.stop();
    }

}
