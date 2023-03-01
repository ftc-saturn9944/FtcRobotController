package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;

public class EncoderDriveSubsystem extends SubsystemBase {
    private final DcMotor m_leftFront;
    private final DcMotor m_rightFront;
    private final DcMotor m_leftRear;
    private final DcMotor m_rightRear;

    private static int m_lfTarget;
    private static int m_lrTarget;
    private static int m_rfTarget;
    private static int m_rrTarget;

    public EncoderDriveSubsystem(DcMotor leftFront, DcMotor leftRear, DcMotor rightFront, DcMotor rightRear) {
        m_leftFront = leftFront;
        m_leftRear = leftRear;
        m_rightFront = rightFront;
        m_rightRear = rightRear;

        m_rightFront.setDirection(DcMotor.Direction.REVERSE);
        m_rightRear.setDirection(DcMotor.Direction.REVERSE);
        m_leftFront.setDirection(DcMotor.Direction.FORWARD);
        m_leftRear.setDirection(DcMotor.Direction.FORWARD);
        m_leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetEncoder() {
        m_leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_lfTarget = 0;
        m_lrTarget = 0;
        m_rfTarget = 0;
        m_rrTarget = 0;
    }


    public boolean atPosition() {
        return (
                !m_leftFront.isBusy() &&
                !m_leftRear.isBusy() &&
                !m_rightFront.isBusy() &&
                !m_rightRear.isBusy());
    }

    public void setEncoder(int distance, String direction) {
        if (direction == "Up") {
            m_lfTarget = distance;
            m_lrTarget = distance;
            m_rfTarget = distance;
            m_rrTarget = distance;
        } else if (direction == "Down") {
            m_lfTarget = -distance;
            m_lrTarget = -distance;
            m_rfTarget = -distance;
            m_rrTarget = -distance;
        } else if (direction == "Left") {
            m_lfTarget = distance;
            m_lrTarget = -distance;
            m_rfTarget = -distance;
            m_rrTarget = distance;
        } else if (direction == "Right") {
            m_lfTarget = -distance;
            m_lrTarget = distance;
            m_rfTarget = distance;
            m_rrTarget = -distance;
        }
        m_leftFront.setTargetPosition(m_lfTarget);
        m_rightFront.setTargetPosition(m_rfTarget);
        m_leftRear.setTargetPosition(m_lrTarget);
        m_rightRear.setTargetPosition(m_rrTarget);

        m_leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void driveEncoder(double power) {
        m_leftFront.setPower(power);
        m_rightFront.setPower(power);
        m_leftRear.setPower(power);
        m_rightRear.setPower(power);
    }
}
