package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class LaunchDrone extends CommandBase {
    private final LauncherSubsystem launcher;

    public LaunchDrone(LauncherSubsystem subsystem) {
        launcher = subsystem;
        addRequirements(launcher);
    }

    public void initialize() {
        launcher.launch();
    }
}
