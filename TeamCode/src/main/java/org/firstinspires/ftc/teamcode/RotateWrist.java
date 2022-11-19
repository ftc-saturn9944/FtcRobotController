package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class RotateWrist extends CommandBase {
    private final WristSubsystem wrist;

    public RotateWrist(WristSubsystem subsystem) {
        wrist = subsystem;
        addRequirements(wrist);
    }

    public void initialize() {
        wrist.rotate();
    }

    public boolean isFinished() {
        return true;
    }
}
