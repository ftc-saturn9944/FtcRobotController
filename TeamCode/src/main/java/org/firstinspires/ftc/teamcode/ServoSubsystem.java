package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoSubsystem extends SubsystemBase {
    private ServoEx servo;
    public ServoSubsystem(HardwareMap hmap, String name){
        servo = new SimpleServo(hmap, name, 0, 180);
    }
    public void setPosition(double position){
        servo.setPosition(position);
    }
    public Double getPosition(){
        return servo.getPosition();
    }

}
