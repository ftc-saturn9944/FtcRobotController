package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class GripperRelease extends CommandBase {
    private final GripperSubsystem gripper;

    public GripperRelease(GripperSubsystem subsystem) {
        gripper = subsystem;
        addRequirements(gripper);
    }

    public void initialize() {
        gripper.release();
    }

    public boolean isFinished() {
        return true;
    }
}
