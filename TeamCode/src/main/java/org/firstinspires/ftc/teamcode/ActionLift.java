package org.firstinspires.ftc.teamcode;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ActionLift {
    private DcMotorEx motor;
    public ActionLift(HardwareMap hMap){

        motor = hMap.get(DcMotorEx.class, "LIFTMOTOR");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public Action increaseHeight(int encoder){
        return new Action() {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    motor.setPower(0.8);
                    initialized = true;
                }
                double pos = motor.getCurrentPosition();
                packet.put("liftpos",pos);
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
                    motor.setPower(-0.8);
                    initialized = true;
                }
                double pos = motor.getCurrentPosition();
                packet.put("liftpos",pos);
                if (pos > encoder) {
                    return true;
                }else {
                    return false;
                }
            }
        };
    }
}
