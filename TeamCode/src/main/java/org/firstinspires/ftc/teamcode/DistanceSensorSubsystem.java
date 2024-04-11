package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorDistance;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorSubsystem extends SubsystemBase {
    private SensorDistance sensor;
    public DistanceSensorSubsystem (HardwareMap hmap, String name){
        sensor = new SensorRevTOFDistance(hmap, name);
    }
    public double getDistance (){
        return sensor.getDistance(DistanceUnit.CM);
    }
}
