package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class TouchSensorSubsystem extends SubsystemBase {
    private TouchSensor sensor;
    public TouchSensorSubsystem (HardwareMap hmap, String name){
        sensor = hmap.get(TouchSensor.class, name);
    }
    public boolean getState(){
        return sensor.isPressed();
    }
}
