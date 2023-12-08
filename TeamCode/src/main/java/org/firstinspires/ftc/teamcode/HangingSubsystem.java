package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HangingSubsystem extends SubsystemBase {
    private final MotorEx hangMotor;

    private Double raisePower = 0.8;
    private Double lowerPower = -0.5;
    public HangingSubsystem(HardwareMap hMap, String motor) {
        hangMotor = new MotorEx(hMap, motor, Motor.GoBILDA.RPM_117);
        hangMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void raiseHang() {
        hangMotor.set(raisePower);
    }

    public void lowerHang() {
        hangMotor.set(lowerPower);
    }

    public void stopHang() {
        hangMotor.stopMotor();
    }
}

