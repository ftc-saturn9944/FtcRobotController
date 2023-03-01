package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class DriveByEncoder extends CommandBase {
    private final EncoderDriveSubsystem m_drive;
    private int m_encoderDist;
    private String m_direction;


    private double m_power;

    public DriveByEncoder(EncoderDriveSubsystem subsystem, int encoderDist, String direction) {
        m_encoderDist = encoderDist;
        m_direction = direction;
        m_drive = subsystem;
        m_power = 0.5;
        addRequirements(m_drive);
    }
    public DriveByEncoder(EncoderDriveSubsystem subsystem, int encoderDist, String direction, double power) {
        m_encoderDist = encoderDist;
        m_direction = direction;
        m_drive = subsystem;
        m_power = power;
        addRequirements(m_drive);
    }

    public void initialize() {
        m_drive.resetEncoder();
        m_drive.setEncoder(m_encoderDist, m_direction);
        m_drive.driveEncoder(m_power);
    }



    public boolean isFinished() {
        return m_drive.atPosition();
    }

    public void end() {
    }

}
