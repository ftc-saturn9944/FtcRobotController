package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;

import java.util.HashMap;

public class DigitalLEDSubsystem extends SubsystemBase {
    private final DigitalChannel green, red;

    public DigitalLEDSubsystem(DigitalChannel green, DigitalChannel red) {
        this.green = green;
        this.red = red;
        this.green.setMode(DigitalChannel.Mode.OUTPUT);
        this.red.setMode(DigitalChannel.Mode.OUTPUT);
    }

    public void setChannels(int channel) {
        switch (channel) {
            case 0:
                green.setState(true);
                red.setState(true);
                break;

            case 1:
                green.setState(false);
                red.setState(true);
                break;

            case 2:
                green.setState(true);
                red.setState(false);
                break;

            case 3:
                green.setState(false);
                red.setState(false);
                break;

        }
    }

    public HashMap<String,Object> getStatus() {
        HashMap<String,Object> out = new HashMap<>();
        out.put("GreenState", green.getState());
        out.put("GreenMode", green.getMode());
        out.put("RedState", red.getState());
        out.put("RedMode", red.getMode());
        return out;
    }
}
