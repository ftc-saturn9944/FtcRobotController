package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class LiftByEncoder extends CommandBase {

    private LiftSubsystem m_lift;
    private int m_position;
    private int minPosition;
    private int maxPosition;

    public LiftByEncoder(LiftSubsystem subsystem, int position) {
        m_lift = subsystem;
        m_position = position;
        minPosition = m_position - 300;
        maxPosition = m_position + 300;
        addRequirements(m_lift);
    }

    public void initialize() {
    }

    @Override
    public void execute() {
        int currPos = m_lift.getEncoderValue();
        if (currPos < m_position) {
            m_lift.raiseLift();
        } else if (currPos > m_position) {
            m_lift.lowerLift();
        }
    }


    public boolean isFinished() {
        int currPos = m_lift.getEncoderValue();
        return currPos >= minPosition && currPos <= maxPosition;
    }
}
