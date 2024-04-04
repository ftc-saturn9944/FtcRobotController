package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

public class LauncherCommand extends CommandBase {
    private ServoSubsystem launcher,cover;
    private double cover1, cover2, launcher1;
    private SequentialCommandGroup sequence;
    private ServoSetPosition c_launcher;
    private ServoTogglePosition c_cover;
    private TimerCommand c_timer;
    public LauncherCommand (
            ServoSubsystem launcher,
            ServoSubsystem cover,
            double cover1,
            double cover2,
            double launcher1,
            long timer
    ) {
        sequence = new SequentialCommandGroup();
        c_cover = new ServoTogglePosition(cover,cover1,cover2);
        c_launcher = new ServoSetPosition(launcher,launcher1);
        c_timer = new TimerCommand(timer);
        addRequirements(cover, launcher);

        sequence.addCommands(
                c_cover,
                c_timer,
                c_launcher,
                c_timer,
                c_cover
        );
    }
    public void initialize() {
        sequence.schedule();
    }

    public double coverTarget() {
        return c_cover.targetPosition();
    }

    public boolean isFinished(){
        return true;
    }
}
