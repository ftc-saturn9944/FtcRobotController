package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ActionGripper {
    private CRServo servo1, servo2;
    public ActionGripper(HardwareMap hMap) {
        servo1 = hMap.get(CRServo.class,"GRIPPER");
        servo2 = hMap.get(CRServo.class,"GRIPPER2");
    }
    public Action intake(double seconds) {
        return new Action() {
            private boolean initialized = false;
            private ElapsedTime time;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    servo1.setPower(0.8);
                    servo2.setPower(-0.8);
                    time = new ElapsedTime();
                    initialized = true;
                }
                double s1p = servo1.getPower();
                double s2p = servo2.getPower();
                telemetryPacket.put("servo1 power", s1p);
                telemetryPacket.put("servo2 power", s2p);
                telemetryPacket.put("elapsed", time.seconds());
                telemetryPacket.put("runfor", seconds);
                if (time.seconds() < seconds) {
                    return true;
                }else {
                    servo1.setPower(0);
                    servo2.setPower(0);
                    return false;
                }
            }
        };
    }
    public Action release(double seconds) {
        return new Action() {
            private boolean initialized = false;
            private ElapsedTime time;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    servo1.setPower(-0.8);
                    servo2.setPower(0.8);
                    time = new ElapsedTime();
                    initialized = true;
                }
                double s1p = servo1.getPower();
                double s2p = servo2.getPower();
                telemetryPacket.put("servo1 power", s1p);
                telemetryPacket.put("servo2 power", s2p);
                telemetryPacket.put("elapsed", time.seconds());
                telemetryPacket.put("runfor", seconds);
                if (time.seconds() < seconds) {
                    return true;
                }else {
                    servo1.setPower(0);
                    servo2.setPower(0);
                    return false;
                }
            }
        };
    }
}
