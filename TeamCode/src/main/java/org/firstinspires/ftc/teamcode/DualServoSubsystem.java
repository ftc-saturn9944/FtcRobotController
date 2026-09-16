package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DualServoSubsystem extends SubsystemBase {
    private ServoEx servo1;
    private ServoEx servo2;
    public DualServoSubsystem(HardwareMap hmap, String name){
        servo1 = new SimpleServo(hmap, name, 0, 180);
        servo2 = new SimpleServo(hmap, name, 0, -180);
    }
}
