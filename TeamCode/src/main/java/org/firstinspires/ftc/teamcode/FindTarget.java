package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class FindTarget extends CommandBase {
    private final DistanceSubsystem distance;
    private DistanceSubsystem.Targets target;
    private DigitalLEDSubsystem d1;
    private double dist;

    public FindTarget(DistanceSubsystem subsystem, DigitalLEDSubsystem digital, DistanceSubsystem.Targets value, double far) {
        distance = subsystem;
        d1 = digital;
        addRequirements(distance, d1);
        target = value;
        dist = far;
    }
    public FindTarget(DistanceSubsystem subsystem, DistanceSubsystem.Targets value, double far) {
        distance = subsystem;
        addRequirements(distance, d1);
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
