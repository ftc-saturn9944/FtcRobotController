package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumSubsystem extends SubsystemBase {

    private final MecanumDrive m_drive;

    public MecanumSubsystem(MotorEx leftFront, MotorEx rightFront, MotorEx leftRear, MotorEx rightRear) {
        m_drive = new MecanumDrive(leftFront, rightFront, leftRear, rightRear);
    }

    public void drive(double strafe, double forward, double turn, boolean square) {
        m_drive.driveRobotCentric(strafe, forward, turn, square);
    }
}
