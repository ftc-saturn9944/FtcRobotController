package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

public class TimerCommand extends CommandBase {
    private Timing.Timer m_timer;

    public TimerCommand(long duration) {
        m_timer = new Timing.Timer(duration, TimeUnit.MILLISECONDS);
    }

    public void initialize() { m_timer.start(); }

    public boolean isFinished() { return m_timer.done(); }
}
