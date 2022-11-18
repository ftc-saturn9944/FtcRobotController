package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.hardware.SensorDistanceEx.DistanceTarget;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.ListIterator;
import java.util.Objects;


class DistanceList {

    private ArrayList<DistanceTarget> targets = new ArrayList<DistanceTarget>();
    private DistanceTarget current;
    public DistanceList () {
    }

    public void addTarget(String name, Double target) {
        targets.add(new DistanceTarget(DistanceUnit.INCH, target, 0.0625, name));
    }
    public void addTarget (String name, Double target, Double threshold) {
        targets.add(new DistanceTarget(DistanceUnit.INCH, target, threshold, name));
    }
    public void addTarget(String name, Double target, @NonNull Boolean asCurrent) {
        targets.add(new DistanceTarget(DistanceUnit.INCH, target, 0.0625, name));
        if (asCurrent) this.setTarget(name);
    }
    public void addTarget (String name, Double target, Double threshold, @NonNull Boolean asCurrent) {
        targets.add(new DistanceTarget(DistanceUnit.INCH, target, threshold, name));
        if (asCurrent) this.setTarget(name);
    }

    public void setTarget (String name) {
        targets.stream()
                .filter(t -> Objects.equals(t.getName(), name))
                .findAny()
                .ifPresent(distanceTarget -> current = distanceTarget);
    }

    public void setTarget (DistanceTarget target) {
        targets.stream()
                .filter(t -> t == target)
                .findAny()
                .ifPresent(distanceTarget -> current = distanceTarget);
    }

    public DistanceTarget getCurrent () {
        return current;
    }

    public void next() {
        int index = targets.indexOf(current);
        ListIterator<DistanceTarget> iter = targets.listIterator(index + 1);
        if (iter.hasNext()) current = iter.next();
    }

    public void previous() {
        int index = targets.indexOf(current);
        ListIterator<DistanceTarget> iter = targets.listIterator(index);
        if (iter.hasPrevious()) current = iter.previous();
    }
}
