package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class RotateDrive extends CommandBase {
    private final MecanumSubsystem drive;

    private double heading, minHeading, maxHeading;

    public RotateDrive(MecanumSubsystem subsystem, double targetHeading) {
        drive = subsystem;
        heading = targetHeading;
        minHeading = heading - 0.3;
        maxHeading = heading + 0.3;
        addRequirements(drive);
    }

    public void execute() {
        if (drive.getHeading() < heading) {
            drive.rotateRight();
        } else if (drive.getHeading() > heading) {
            drive.rotateLeft();
        }
    }

    public boolean isFinished() {
        double currHeading = drive.getHeading();
        return currHeading >= minHeading && currHeading <= maxHeading;
    }
}
