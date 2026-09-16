package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class CRServoStop extends CommandBase {
    private CRServoSubsystem servo;

    public CRServoStop(CRServoSubsystem subsystem){
        servo = subsystem;
        addRequirements(servo);
    }
    public void execute(){
        servo.stop();
    }
}
