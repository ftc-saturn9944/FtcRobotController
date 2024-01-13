package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class GripperRelease extends CommandBase {
    private final GripperSubsystem gripper;

    public GripperRelease(GripperSubsystem subsystem) {
        gripper = subsystem;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
        gripper.release();
    }

    public boolean isFinished() {
        return true;
    }
}
