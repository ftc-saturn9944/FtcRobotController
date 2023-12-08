package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftSubsystem extends SubsystemBase {

    //private final SensorRevTOFDistance liftSensor;
    private final MotorEx liftMotor;

    private DistanceList scoring;

    private Double MOTOR_POWER = 1.0;
    public LiftSubsystem(HardwareMap hMap, String motor) {
        //liftSensor = new SensorRevTOFDistance(hMap, lift);
        liftMotor = new MotorEx(hMap, motor);

        liftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        scoring = new DistanceList();
        scoring.addTarget("Junction", 13.0, 2.0,true);
        scoring.addTarget("Low", 25.0, 2.0);
        scoring.addTarget("Mid", 40.0, 2.0);
        scoring.addTarget("High", 60.0, 2.0);
    }

    public void raise() {
        scoring.next();
    }

    public void lower() {
        scoring.previous();
    }
/*
    public void moveLift() {
        if (this.targetReached()) {
            liftMotor.set(0);
        } else if (this.getDistance() < scoring.getCurrent().getTarget()) {
            liftMotor.set(MOTOR_POWER);
        } else {
            liftMotor.set(-MOTOR_POWER);
        }
    }
*/
    public void setPosition(int position) {
        liftMotor.setTargetPosition(position);
    }

    public void setPower(double power) {
        liftMotor.set(power);
    }

    public void raiseLift() {
        liftMotor.set(-MOTOR_POWER);
    }

    public void lowerLift() {
        liftMotor.set(MOTOR_POWER);
    }

    public void stopLift() { liftMotor.set(-0.05); }
    public String getTargetName() {
        return scoring.getCurrent().getName();
    }

    public Double getTargetDist() {
        return scoring.getCurrent().getTarget();
    }
    public int getEncoderValue() {return liftMotor.getCurrentPosition();}

    /*
    public boolean targetReached() {
        return scoring.getCurrent().atTarget(liftSensor.getDistance(DistanceUnit.CM));
    }
    public double getDistance() {
        return liftSensor.getDistance(DistanceUnit.CM);
    }

    */
}
