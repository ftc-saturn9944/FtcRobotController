package org.firstinspires.ftc.teamcode;

import android.content.ContentQueryMap;

import com.arcrobotics.ftclib.command.CommandBase;

public class CRServoBackwardDual extends CommandBase {
    private CRServoSubsystem servo, servo2;

    public CRServoBackwardDual(CRServoSubsystem subsystem, CRServoSubsystem subsystem2){
        servo = subsystem;
        servo2 = subsystem2;
        addRequirements(servo, servo2);
    }
    public void execute(){

        servo.backward();
        servo2.backward();
    }
    public void end(){

        servo.stop();
        servo2.stop();
    }

}
