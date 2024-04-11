package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class DigitalLEDSetChannel extends CommandBase {
    private DigitalLEDSubsystem digital;
    private int channel;
    public DigitalLEDSetChannel(DigitalLEDSubsystem digital, int channel){
        this.digital = digital;
        this.channel = channel;
        addRequirements(digital);

    }
    public void initialize(){
        digital.setChannels(channel);
    }
    public boolean isFinished (){
        return true;

    }
}
