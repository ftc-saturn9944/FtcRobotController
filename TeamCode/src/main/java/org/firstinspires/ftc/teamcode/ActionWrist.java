package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ActionWrist {
    private Servo wrist;
    private double score, center;
    public ActionWrist(HardwareMap hMap, double score, double center){
        wrist = hMap.get(Servo.class, "WRIST");
        this.score = score;
        this.center = center;
    }
    public Action score() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist.setPosition(score);
                return false;
            }
        };

    }
    public Action center() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                wrist.setPosition(center);
                return false;
            }
        };

    }

}
