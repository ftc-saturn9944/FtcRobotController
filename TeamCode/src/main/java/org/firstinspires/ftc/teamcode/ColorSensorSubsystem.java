package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SensorColor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

public class ColorSensorSubsystem extends SubsystemBase {
    private RevColorSensorV3 sensor;
    public ColorSensorSubsystem (HardwareMap hmap, String name){
        sensor = hmap.get(RevColorSensorV3.class, name);
    }

    public double getDistance(){
        return sensor.getDistance(DistanceUnit.CM);
    }

    public int getColor(){
        return sensor.argb();
    }
    public ArrayList<Integer>getColors(){
        ArrayList<Integer> data = new ArrayList<Integer>();
        data.add(sensor.alpha());
        data.add(sensor.red());
        data.add(sensor.green());
        data.add(sensor.blue());
        return data;
    }
}
