package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.RevIMU;

public class RotateDrive extends CommandBase {
    private final MecanumSubsystem drive;
    private final RevIMU m_imu;

    private double heading, minHeading, maxHeading;

    public RotateDrive(MecanumSubsystem subsystem, RevIMU imu, double targetHeading) {
        drive = subsystem;
        m_imu = imu;
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

    public void end(boolean interrupted) {
        drive.drive(0,0,0, false, m_imu, false);
    }
}
