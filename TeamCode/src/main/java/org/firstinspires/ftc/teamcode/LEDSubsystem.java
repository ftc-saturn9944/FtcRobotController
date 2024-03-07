package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDSubsystem extends SubsystemBase {
    private RevBlinkinLedDriver led;

    public LEDSubsystem(HardwareMap hmap, String name){
        led = hmap.get(RevBlinkinLedDriver.class, name);
    }
    public void setLed(RevBlinkinLedDriver.BlinkinPattern pattern){
        led.setPattern(pattern);
    }

}
