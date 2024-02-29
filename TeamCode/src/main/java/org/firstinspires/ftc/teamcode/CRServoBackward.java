package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class CRServoBackward extends CommandBase {
    private CRServoSubsystem servo;

    public CRServoBackward(CRServoSubsystem subsystem){
        servo = subsystem;
        addRequirements(servo);
    }
    public void execute(){
        servo.backward();
    }
    public void end(){
        servo.stop();
    }

}
