package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class CRServoStopDual extends CommandBase {
    private CRServoSubsystem servo, servo2;

    public CRServoStopDual(CRServoSubsystem subsystem, CRServoSubsystem subsystem2){
        servo = subsystem;
        servo2 = subsystem2;
        addRequirements(servo2, servo);
    }
    public void execute(){
        servo2.stop();
        servo.stop();
    }
}
