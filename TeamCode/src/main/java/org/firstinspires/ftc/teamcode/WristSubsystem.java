package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WristSubsystem extends SubsystemBase {
    private final ServoEx gripper;

    private Double wristPosition = 0.7;
    public WristSubsystem(final HardwareMap hMap, final String name) {
        gripper = hMap.get(ServoEx.class, name);
    }

    public WristSubsystem(ServoEx wrist) {
        gripper = wrist;
    }
    public void rotate() {
        wristPosition = wristPosition == 0.02 ? 0.7 : 0.02;//.7 is back
        gripper.setPosition(wristPosition);
    }
}
