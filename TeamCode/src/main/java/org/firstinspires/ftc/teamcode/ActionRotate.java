package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ActionRotate {
    private DcMotorEx motor;
    public ActionRotate(HardwareMap hMap){
        motor = hMap.get(DcMotorEx.class, "ROTATE");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public Action increaseHeight(int encoder){
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setPower(1.0);
                    initialized = true;
                }
                double pos = motor.getCurrentPosition();
                packet.put("rotatepos",pos);
                if (pos < encoder) {
                    return true;
                }else {
                    motor.setPower(0);
                    return false;
                }
            }
        };
    }
    public Action decreaseHeight(int encoder){
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setPower(-1.0);
                    initialized = true;
                }
                double pos = motor.getCurrentPosition();
                packet.put("liftpos",pos);
                if (pos > encoder) {
                    return true;
                }else {
                    motor.setPower(0);
                    return false;
                }
            }
        };
    }
}
