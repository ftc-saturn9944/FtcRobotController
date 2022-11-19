package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GripperSubsystem extends SubsystemBase {

    private final ServoEx gripper;

    public GripperSubsystem(final HardwareMap hMap, final String name) {
        gripper = hMap.get(ServoEx.class, name);
    }

    public GripperSubsystem(ServoEx gripper) {
        this.gripper = gripper;
    }

    public void grab() {
        gripper.setPosition(0.8);
    }

    public void release() {
        gripper.setPosition(0.2);
    }

    public double getPosition () {
        return gripper.getPosition();
    }
}
