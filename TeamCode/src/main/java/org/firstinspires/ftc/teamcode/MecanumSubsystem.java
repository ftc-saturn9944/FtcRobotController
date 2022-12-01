package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumSubsystem extends SubsystemBase {

    private final MecanumDrive m_drive;
    private final RevIMU m_imu;
    private final Boolean m_field;


    public MecanumSubsystem(MotorEx leftFront, MotorEx rightFront, MotorEx leftRear, MotorEx rightRear, RevIMU imu, Boolean field) {
        m_drive = new MecanumDrive(leftFront, rightFront, leftRear, rightRear);
        m_imu = imu;
        m_field = field;
    }

    public void drive(double strafe, double forward, double turn, boolean square, RevIMU imu, Boolean field) {
        if (field) {
            m_drive.driveFieldCentric(strafe, forward, turn, imu.getRotation2d().getDegrees(), square);
        } else {
            m_drive.driveRobotCentric(strafe, forward, turn, square);
        }
    }
}
