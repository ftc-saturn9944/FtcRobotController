package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

public class AutonomousForward extends CommandBase {
    private final MecanumSubsystem m_drive;
    private double m_strafe, m_forward, m_rotation;
    private Timing.Timer m_timer;
    private long m_duration;
    private String m_direction;
    private RevIMU m_imu;
    private Boolean m_field;
    private CameraSubsystem m_camera;

    private double POWER = 0.5;

    public AutonomousForward(MecanumSubsystem subsystem, CameraSubsystem camera, RevIMU imu, Boolean field) {
        m_drive = subsystem;
        m_field = field;
        m_imu = imu;
        m_camera = camera;
        addRequirements(m_drive);
    }

    public void initialize() {
        m_timer = new Timing.Timer(m_camera.getForwardTime(), TimeUnit.MILLISECONDS);
        m_timer.start();
    }

    @Override
    public void execute() {
        m_strafe = 0;
        m_forward = -POWER;
        m_rotation = 0;
        m_drive.drive(m_strafe, m_forward, m_rotation, false, m_imu, m_field);
    }


    public boolean isFinished() {
        return m_timer.done();
    }

    public void end() {
        m_drive.drive(0,0,0, false, m_imu, m_field);
    }
}

