package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class GripperStop extends CommandBase {

    private final GripperSubsystem gripper;

    public GripperStop(GripperSubsystem subsystem) {
        gripper = subsystem;
        addRequirements(gripper);
    }

    public void initialize() {
        gripper.stop();
    }
    public boolean isFinished() { return true; };
}
