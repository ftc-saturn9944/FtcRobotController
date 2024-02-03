package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class SetLEDPattern extends CommandBase {
    private LEDSubsystem led;
    private RevBlinkinLedDriver.BlinkinPattern pattern;

    public SetLEDPattern(LEDSubsystem subsystem, RevBlinkinLedDriver.BlinkinPattern pattern) {
        led = subsystem;
        this.pattern = pattern;
        addRequirements(led);
    }

    public void initialize() {
        led.setLed(pattern);
    }
}
