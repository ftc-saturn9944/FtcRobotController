package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveTest extends SequentialCommandGroup {

    public DriveTest(MecanumSubsystem drive, GripperSubsystem gripper, RevIMU imu) {
        addCommands(
                new GripperGrab(gripper),
/* Terminal/Substation */
                new DriveSeconds(drive, 0, "stop", imu, false),
                new DriveSeconds(drive, 200, "up", imu, false),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new DriveSeconds(drive, 2300, "left", imu, false),

                /* Otherwise */
//                new DriveSeconds(drive, 0, "stop", imu, false),
//                new DriveSeconds(drive, 400, "right", imu, false),
//                new DriveSeconds(drive, 0, "stop", imu, false),
//                new DriveSeconds(drive, 1200, "up", imu, false),
//                new DriveSeconds(drive, 0, "stop", imu, false),
//                new DriveSeconds(drive, 250, "down", imu, false),
//                new DriveSeconds(drive, 0, "stop", imu, false),
//                new DriveSeconds(drive, 2000, "right", imu, false),
                //new DriveSeconds(drive, 2200, "left", imu, false),
                new DriveSeconds(drive, 0, "stop", imu, false),
                new GripperRelease(gripper)
        );
        addRequirements(drive, gripper);
    }
}
