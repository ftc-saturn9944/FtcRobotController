package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class DualMotorLower extends CommandBase {
    private DualMotorSubsystem motor;

    public DualMotorLower(DualMotorSubsystem subsystem){
        motor = subsystem;
        addRequirements(motor);
    }
    public void execute(){
        motor.lower();
    }
    public void end(){
        motor.stop();
    }

}
