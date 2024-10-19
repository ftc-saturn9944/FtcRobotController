package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.RevIMU;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
    private final MecanumSubsystem m_drive;
    private DoubleSupplier m_forward;
    private DoubleSupplier m_strafe;
    private DoubleSupplier m_rotation;
    private RevIMU m_imu;
    private Boolean m_field;

    public DefaultDrive(MecanumSubsystem subsystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier rotation, RevIMU imu, Boolean field) {
        m_drive = subsystem;
        m_forward = forward;
        m_strafe = strafe;
        m_rotation = rotation;
        m_imu = imu;
        m_field = field;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(m_strafe.getAsDouble(), m_forward.getAsDouble(), m_rotation.getAsDouble(), false, m_imu, m_field);
    }
}
