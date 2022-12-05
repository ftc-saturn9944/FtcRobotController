package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTest extends SequentialCommandGroup {

    public DriveTest(MecanumSubsystem drive, GripperSubsystem gripper, RevIMU imu) {
        addCommands(
                new DriveSeconds(drive, 0, "stop", imu, false),
                new DriveSeconds(drive, 1, "up", imu, false),
                new DriveSeconds(drive, 1, "right", imu, false),
                new DriveSeconds(drive, 1, "down", imu, false),
                new DriveSeconds(drive, 3, "left", imu, false),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new GripperRelease(gripper)
        );
        addRequirements(drive, gripper);
    }
}
