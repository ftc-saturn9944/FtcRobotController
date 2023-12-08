package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WristSubsystem extends SubsystemBase {
    private final ServoEx gripper;

    private Double scoring = 0.7;
    private Double moving = 0.44;
    private Double wristPosition = moving;
    public WristSubsystem(final HardwareMap hMap, final String name) {
        gripper = hMap.get(ServoEx.class, name);
    }

    public WristSubsystem(ServoEx wrist) {
        gripper = wrist;
    }
    public void rotate() {
        wristPosition = wristPosition == moving ? scoring : moving;
        gripper.setPosition(wristPosition);
    }

    public double getPosition () {
        return gripper.getPosition();
    }
}
