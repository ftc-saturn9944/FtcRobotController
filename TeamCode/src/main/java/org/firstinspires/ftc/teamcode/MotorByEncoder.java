package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class MotorByEncoder extends CommandBase {

    private final MotorSubsystem m_Motor;
    private final int m_position;
    private final int minPosition;
    private final int maxPosition;

    public MotorByEncoder(MotorSubsystem subsystem, int position) {
        m_Motor = subsystem;
        m_position = position;
        minPosition = m_position - 10;
        maxPosition = m_position + 10;
        addRequirements(m_Motor);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        int currPos = m_Motor.getEncoder();
        if (currPos < m_position) {
            m_Motor.raise();
        } else if (currPos > m_position) {
            m_Motor.lower();
        }
    }

    @Override
    public boolean isFinished() {
        int currPos = m_Motor.getEncoder();
        return currPos >= minPosition && currPos <= maxPosition;
    }


    public void end(boolean interrupted) {
        m_Motor.stop();
    }
}
