package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {
    private final MecanumSubsystem m_drive;
    private DoubleSupplier m_forward;
    private DoubleSupplier m_strafe;
    private DoubleSupplier m_rotation;

    public DefaultDrive(MecanumSubsystem subsystem, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier rotation) {
        m_drive = subsystem;
        m_forward = forward;
        m_strafe = strafe;
        m_rotation = rotation;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.drive(m_strafe.getAsDouble(), m_forward.getAsDouble(), m_rotation.getAsDouble(), false);
    }
}
