package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class CRServoForwardDual extends CommandBase {
    private CRServoSubsystem servo, servo2;

    public CRServoForwardDual(CRServoSubsystem subsystem, CRServoSubsystem subsystem2){
        servo = subsystem;
        servo2 = subsystem2;
        addRequirements(servo, servo2);
    }
    public void execute(){

        servo.forward();
        servo2.forward();
    }
    public void end(){

        servo.stop();
        servo2.stop();
    }

}
