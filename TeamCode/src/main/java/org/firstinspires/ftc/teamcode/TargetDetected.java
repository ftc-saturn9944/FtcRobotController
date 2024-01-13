package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class TargetDetected extends CommandBase {
    private final DistanceSubsystem distance;

    public TargetDetected (DistanceSubsystem subsystem) {
        distance = subsystem;
    }

    public void execute() {
        distance.getTarget();
    }

    public boolean isFinished() {
        return distance.getTarget().name() != "None";
    }
}
