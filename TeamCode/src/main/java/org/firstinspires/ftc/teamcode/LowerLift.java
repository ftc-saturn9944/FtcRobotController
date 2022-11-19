package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class LowerLift extends CommandBase {
    private final LiftSubsystem m_lift;

    public LowerLift(LiftSubsystem subsystem) {
        m_lift = subsystem;
        addRequirements(m_lift);
    }

//    @Override
//    public void initialize() {
//        m_lift.lower();
//    }
    @Override
    public void execute() {
        m_lift.lowerLift();
    }

//    @Override
//    public boolean isFinished() {
//        return m_lift.targetReached();
//    }
}
