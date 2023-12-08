package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class HangStop extends CommandBase {
    private final HangingSubsystem m_hang;

    public HangStop(HangingSubsystem subsystem) {
        m_hang = subsystem;
        addRequirements(m_hang);
    }
    @Override
    public void execute() {
        m_hang.stopHang();
    }
}
