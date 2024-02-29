package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CRServoSubsystem extends SubsystemBase {
    private Double forwardSpeed,backwardSpeed;
    private CRServo servo;
    public CRServoSubsystem(HardwareMap hMap,String name,Double forwardSpeed,Double backwardSpeed){
        this.forwardSpeed = forwardSpeed;
        this.backwardSpeed = backwardSpeed;
        servo = new CRServo(hMap, name);
    }
    public void forward(){
        servo.set(forwardSpeed);
    }
    public void backward(){
        servo.set(backwardSpeed);
    }
    public void stop(){
        servo.stopMotor();
    }
}
