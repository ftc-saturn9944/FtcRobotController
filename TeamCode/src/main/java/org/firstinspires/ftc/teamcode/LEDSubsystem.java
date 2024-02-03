package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class LEDSubsystem extends SubsystemBase {

    private final RevBlinkinLedDriver led;

    public LEDSubsystem (RevBlinkinLedDriver lights) {
        led = lights;
    }

    public void setLed(RevBlinkinLedDriver.BlinkinPattern pattern) {
        led.setPattern(pattern);
    }
}
