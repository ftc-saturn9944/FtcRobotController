package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


public class RotateWrist extends CommandBase {
    private final WristSubsystem wrist;
    private final LEDSubsystem leds;
    private final RevBlinkinLedDriver.BlinkinPattern pattern;
    private double position;
    public RotateWrist(WristSubsystem subsystem, LEDSubsystem leds, double position, RevBlinkinLedDriver.BlinkinPattern pattern) {
        wrist = subsystem;
        this.leds = leds;
        this.position = position;
        this.pattern = pattern;
        addRequirements(wrist, leds);
    }

    public void initialize() {
        leds.setLed(pattern);
        wrist.rotate(position);
    }

    public boolean isFinished() {
        return true;
    }
}
