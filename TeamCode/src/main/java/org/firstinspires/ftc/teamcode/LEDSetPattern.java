package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class LEDSetPattern extends CommandBase {
    private LEDSubsystem led;
    private RevBlinkinLedDriver.BlinkinPattern pattern;
    public LEDSetPattern(LEDSubsystem subsystem, RevBlinkinLedDriver.BlinkinPattern pattern){
        led = subsystem;
        this.pattern = pattern;
        addRequirements(subsystem);
    }
    public void initialize(){
        led.setLed(pattern);
    }
}
