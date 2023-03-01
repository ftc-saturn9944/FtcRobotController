package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class StopLiftAutonomous extends CommandBase {
    private final LiftSubsystem m_lift;

    public StopLiftAutonomous(LiftSubsystem subsystem) {
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

    public boolean isFinished() { return true; }
}