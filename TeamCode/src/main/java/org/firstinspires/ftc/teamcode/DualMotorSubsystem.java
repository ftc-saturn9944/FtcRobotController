package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DualMotorSubsystem extends SubsystemBase {
    private Double liftPower, lowerPower, stopPower;
    private MotorEx motor, motor2;

    //intialization
    public DualMotorSubsystem(HardwareMap hmap, String name, String name2, double liftPower, double lowerPower, double stopPower, Motor.GoBILDA type, Motor.ZeroPowerBehavior ZeroPower){
        this.liftPower = liftPower;
        this.lowerPower = lowerPower;
        this.stopPower = stopPower;
        motor = new MotorEx(hmap, name, type);
        motor2 = new MotorEx(hmap, name2, type);
        motor.setZeroPowerBehavior(ZeroPower);
        motor2.setZeroPowerBehavior(ZeroPower);

    }

    public int getEncoder() {
        return motor.getCurrentPosition();
    }
    public void raise(){
        motor.set(liftPower);
        motor2.set(-liftPower);
    }
    public void lower(){
        motor.set(lowerPower);
        motor2.set(-lowerPower);
    }
    public void stop(){
        if (stopPower == 0){
            motor.stopMotor();
            motor2.stopMotor();
        }
        else {
            motor.set(stopPower);
            motor2.set(-stopPower);
        }
    }

    public void resetEncoder() {
        motor.resetEncoder();
        motor2.resetEncoder();
    }
    public double getMotorSpeed (){
        return motor.getVelocity();
    }
}
