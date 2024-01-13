package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class GripperGrab extends CommandBase {
    private final GripperSubsystem gripper;

    public GripperGrab(GripperSubsystem subsystem) {
        gripper = subsystem;
        addRequirements(gripper);
    }

    @Override
    public void execute() {
        gripper.grab();
    }

    //public boolean isFinished() {
    //    return true;
    //}
}
