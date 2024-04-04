package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

public class TimerCommand extends CommandBase {
    private Timing.Timer timer;
    public TimerCommand (long duration){
        timer=new Timing.Timer(duration, TimeUnit.MILLISECONDS);
    }
    public void initialize(){
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
