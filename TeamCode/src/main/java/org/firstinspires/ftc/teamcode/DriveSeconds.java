package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;
import java.util.function.DoubleSupplier;

public class DriveSeconds extends CommandBase {
    private final MecanumSubsystem m_drive;
    private double m_strafe, m_forward, m_rotation;
    private Timing.Timer m_timer;
    private double m_duration;
    private String m_direction;
    private RevIMU m_imu;
    private Boolean m_field;

    private double m_power;

    public DriveSeconds(MecanumSubsystem subsystem, long duration, String direction, RevIMU imu, Boolean field, double power) {
        m_timer = new Timing.Timer(duration, TimeUnit.MILLISECONDS);
        m_duration = duration;
        m_direction = direction;
        m_drive = subsystem;
        m_field = field;
        m_imu = imu;
        m_power = power;
        addRequirements(m_drive);
    }
    public DriveSeconds(MecanumSubsystem subsystem, long duration, String direction, RevIMU imu, Boolean field) {
        m_timer = new Timing.Timer(duration, TimeUnit.MILLISECONDS);
        m_duration = duration;
        m_direction = direction;
        m_drive = subsystem;
        m_field = field;
        m_imu = imu;
        m_power = 0.5;
        addRequirements(m_drive);
    }

    public void initialize() {
        m_timer.start();
    }

    @Override
    public void execute() {
        switch (m_direction) {
            case "left":
                m_strafe = -m_power;
                m_forward = 0;
                m_rotation = 0;
                break;
            case "right":
                m_strafe = +m_power;
                m_forward = 0;
                m_rotation = 0;
                break;
            case "up":
                m_strafe = 0;
                m_forward = -m_power;
                m_rotation = 0;
                break;
            case "down":
                m_strafe = 0;
                m_forward = +m_power;
                m_rotation = 0;
                break;
            case "stop":
                m_strafe = 0;
                m_forward = 0;
                m_rotation = 0;
                break;
        }
        m_drive.drive(m_strafe, m_forward, m_rotation, false, m_imu, m_field);
    }


    public boolean isFinished() {
        return m_timer.done();
    }

    public void end() {
        m_drive.drive(0,0,0, false, m_imu, m_field);
    }

}
