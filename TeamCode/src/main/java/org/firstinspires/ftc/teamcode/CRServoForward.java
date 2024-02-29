package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class CRServoForward extends CommandBase {
    private CRServoSubsystem servo;

    public CRServoForward(CRServoSubsystem subsystem){
        servo = subsystem;
        addRequirements(servo);
    }
    public void execute(){
        servo.forward();
    }
    public void end(){
        servo.stop();
    }

}
