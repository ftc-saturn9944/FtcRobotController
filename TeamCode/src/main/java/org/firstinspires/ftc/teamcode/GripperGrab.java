package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class GripperGrab extends CommandBase {
    private final GripperSubsystem gripper;

    public GripperGrab(GripperSubsystem subsystem) {
        gripper = subsystem;
        addRequirements(gripper);
    }

    public void initialize() {
        gripper.grab();
    }

    public void release() {
        gripper.release();
    }

    public boolean isFinished() {
        return true;
    }
}
