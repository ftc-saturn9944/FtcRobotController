package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class MecanumSubsystem extends SubsystemBase {

    private final MecanumDrive m_drive;
    private final RevIMU m_imu;
    private static Boolean m_field;
    private static MotorEx m_leftFront;
    private static MotorEx m_leftRear;
    private static MotorEx m_rightFront;
    private static MotorEx m_rightRear;
    public MecanumSubsystem(MotorEx leftFront, MotorEx rightFront, MotorEx leftRear, MotorEx rightRear, RevIMU imu, Boolean field) {
        m_leftFront = leftFront;
        m_leftRear = leftRear;
        m_rightFront = rightFront;
        m_rightRear = rightRear;
        m_drive = new MecanumDrive(leftFront, rightFront, leftRear, rightRear);
        m_imu = imu;
        m_field = field;
    }

    public void swapDriveMethod() {
        m_field = !m_field;
    }

    public Boolean getDriveMethod() {
        return m_field;
    }

    public void encoderDrive(double power) {
        m_leftFront.set(power);
        m_leftRear.set(power);
        m_rightFront.set(power);
        m_rightRear.set(power);
    }

    public boolean atPosition() {
        return m_rightRear.atTargetPosition() && m_rightFront.atTargetPosition() && m_leftFront.atTargetPosition() && m_leftRear.atTargetPosition();
    }
    public void resetEncoder() {
        m_leftRear.resetEncoder();
        m_leftFront.resetEncoder();
        m_rightFront.resetEncoder();
        m_rightRear.resetEncoder();
    }

    public void setEncoder(int encoderDist, String direction) {
        this.resetEncoder();

        if (direction == "Up") {
            m_leftFront.setTargetPosition(encoderDist);
            m_rightFront.setTargetPosition(encoderDist);
            m_leftRear.setTargetPosition(encoderDist);
            m_rightRear.setTargetPosition(encoderDist);
        }
        else if (direction == "Down") {
            m_leftFront.setTargetPosition(-encoderDist);
            m_rightFront.setTargetPosition(-encoderDist);
            m_leftRear.setTargetPosition(-encoderDist);
            m_rightRear.setTargetPosition(-encoderDist);
        }
        else if (direction == "Left") {

        }
        else if (direction == "Right") {

        } else if (direction == "Stop") {
            m_leftFront.setTargetPosition(0);
            m_rightFront.setTargetPosition(-encoderDist);
            m_leftRear.setTargetPosition(-encoderDist);
            m_rightRear.setTargetPosition(-encoderDist);
        }
    }
    public void drive(double strafe, double forward, double turn, boolean square, RevIMU imu, Boolean field) {
        if (field) {
            m_drive.driveFieldCentric(strafe, forward, turn, imu.getRotation2d().getDegrees(), square);
        } else {
            m_drive.driveRobotCentric(strafe, forward, turn, square);
        }
    }


}
