package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class StopLift extends CommandBase {
    private final LiftSubsystem m_lift;

    public StopLift(LiftSubsystem subsystem) {
        m_lift = subsystem;
        addRequirements(m_lift);
    }

    //    @Override
//    public void initialize() {
//        m_lift.raise();
//    }
    @Override
    public void execute() {
        m_lift.stopLift();
    }

//    @Override
//    public boolean isFinished() {
//        return m_lift.targetReached();
//    }
}
