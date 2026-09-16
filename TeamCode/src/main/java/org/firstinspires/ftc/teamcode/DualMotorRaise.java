package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class DualMotorRaise extends CommandBase {
    private DualMotorSubsystem motor;

    public DualMotorRaise(DualMotorSubsystem subsystem){
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
