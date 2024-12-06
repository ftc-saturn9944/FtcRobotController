package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

public class CRServoForwardDualTimed extends CommandBase {
    private CRServoSubsystem servo, servo2;
    private Timing.Timer timer;

    public CRServoForwardDualTimed(
            CRServoSubsystem subsystem,
            CRServoSubsystem subsystem2,
            long duration
    ){
        servo = subsystem;
        servo2 = subsystem2;
        timer = new Timing.Timer(duration, TimeUnit.MILLISECONDS);
        addRequirements(servo, servo2);
    }

    public void initialize() {
        timer.start();
    }
    public void execute(){

        servo.forward();
        servo2.forward();
    }

    public boolean isFinished(){
        return timer.done();
    }
    public void end(boolean interrupted){

        servo.stop();
        servo2.stop();
    }

}
