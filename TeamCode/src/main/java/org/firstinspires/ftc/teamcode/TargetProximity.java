package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class TargetProximity extends CommandBase {
    private final DistanceSubsystem distance;
    private double dist;

    public TargetProximity(DistanceSubsystem subsystem, double far) {
        distance = subsystem;
        addRequirements(distance);
        dist = far;
    }

    public void execute() {
        distance.getDistance();
    }

    @Override
    public boolean isFinished() {
        return distance.getDistance() < dist;
    }
}
