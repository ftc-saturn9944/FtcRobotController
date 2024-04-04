package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class ServoTogglePosition extends CommandBase {
    private ServoSubsystem servo;
    private Double position1, position2, target;
    public ServoTogglePosition(ServoSubsystem subsystem, Double position1, Double position2){
        servo = subsystem;
        this.position1 = position1;
        this.position2 = position2;
        target = position1;
        addRequirements(subsystem);
    }
    public void initialize(){
        double precision = 0.0001;
        if (Math.abs(servo.getPosition()-position1) < precision){
            target = position2;
        }
        else if (Math.abs(servo.getPosition()-position2) < precision){
            target = position1;
        }
        else{
            target = position1;
        }
        servo.setPosition(target);
    }
    public boolean isFinished(){
        return true;
    }

    public Double targetPosition(){
        return target;
    }
}
