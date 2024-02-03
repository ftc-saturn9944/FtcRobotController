package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LauncherSubsystem  extends SubsystemBase {
    private final ServoEx gripper;

    private Double launchPosition = 0.1;

    public LauncherSubsystem(final HardwareMap hMap, final String name)  {
        gripper = new SimpleServo(hMap, name, 0, 180);
    }

    public void launch() {
        gripper.setPosition(launchPosition);
    }
}
