package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GripperSubsystem extends SubsystemBase {

    private final CRServo gripper;

    public GripperSubsystem(final HardwareMap hMap, final String name) {
        gripper = hMap.get(CRServo.class, name);
    }

    public GripperSubsystem(CRServo gripper) {

        this.gripper = gripper;
        //this.gripper.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void grab() {
        gripper.set(1.0);
    }

    public void release() {
        gripper.set(-1.0);
    }

    public void stop() {gripper.stopMotor();}
}
