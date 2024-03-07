package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class ServoSetPosition extends CommandBase {
    private ServoSubsystem servo;
    private Double position;
    public ServoSetPosition(ServoSubsystem subsystem, Double position){
        servo = subsystem;
        this.position = position;
        addRequirements(subsystem);
    }
    public void initialize(){
        servo.setPosition(position);
    }
    public boolean isFinished(){
        return true;
    }
}
