package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorSubsystem extends SubsystemBase {
    private Double liftPower, lowerPower, stopPower;
    private MotorEx motor;

    //intialization
    public MotorSubsystem(HardwareMap hmap, String name, double liftPower, double lowerPower, double stopPower, Motor.GoBILDA type, Motor.ZeroPowerBehavior ZeroPower){
        this.liftPower = liftPower;
        this.lowerPower = lowerPower;
        this.stopPower = stopPower;
        motor = new MotorEx(hmap, name, type);
        motor.setZeroPowerBehavior(ZeroPower);

    }
    public void raise(){
        motor.set(liftPower);
    }
    public void lower(){
        motor.set(lowerPower);
    }
    public void stop(){
        if (stopPower == 0){
            motor.stopMotor();
        }
        else {
            motor.set(stopPower);
        }
    }
}
