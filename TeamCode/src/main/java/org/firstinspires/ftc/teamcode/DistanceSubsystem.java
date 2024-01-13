package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorDistance;
import com.arcrobotics.ftclib.hardware.SensorRevTOFDistance;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSubsystem extends SubsystemBase {
    private final SensorDistance sensor;

    private Targets target = Targets.None;

    public enum Targets {
        None,
        Left,
        Center,
        Right
    }
    public DistanceSubsystem(final HardwareMap hMap, final String name) {
        sensor = new SensorRevTOFDistance(hMap, name);
    }
    public double getDistance(){
        return sensor.getDistance(DistanceUnit.CM);
    }

    public void setTarget(Targets value){
        target = value;
    }

    public Targets getTarget(){
        return target;
    }


}
