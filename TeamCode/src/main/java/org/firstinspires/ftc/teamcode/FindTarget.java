package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class FindTarget extends CommandBase {
    private final DistanceSubsystem distance;
    private DistanceSubsystem.Targets target;
    private double dist;

    public FindTarget(DistanceSubsystem subsystem, DistanceSubsystem.Targets value, double far) {
        distance = subsystem;
        addRequirements(distance);
        target = value;
        dist = far;
    }

    @Override
    public void initialize() {
        if (distance.getDistance() < dist) {
            distance.setTarget(target);
        }
    }
}
