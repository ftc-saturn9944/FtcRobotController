package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class MotorLower extends CommandBase {
    private MotorSubsystem motor;

    public MotorLower(MotorSubsystem subsystem){
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
