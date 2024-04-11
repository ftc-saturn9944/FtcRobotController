package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Objects;

public class DigitalLEDSubsystem extends SubsystemBase {
    private final DigitalChannel green, red;
    public DigitalLEDSubsystem (final HardwareMap hmap, String greenChannel, String redChannel){
        green = hmap.get(DigitalChannel.class, greenChannel);
        red = hmap.get(DigitalChannel.class, redChannel);
        green.setMode(DigitalChannel.Mode.OUTPUT);
        red.setMode(DigitalChannel.Mode.OUTPUT);
    }
    public void setChannels (int channel){
        // setState(true) means turn off the channel
        // setState(false) means turn on the channel
        switch (channel){
            case 0: //off
                green.setState(true);
                red.setState(true);
                break;
            case 1: //green
                green.setState(false);
                red.setState(true);
                break;
            case 2: //red
                green.setState(true);
                red.setState(false);
                break;
            case 3: //yellow
                green.setState(false);
                red.setState(false);
                break;
        }
    }
    public HashMap<String, Object> getStatus(){
        HashMap<String, Object> out = new HashMap<>();
        out.put("GreenState", green.getState());
        out.put("GreenMode", green.getMode());
        out.put("RedState", red.getState());
        out.put("RedMode", red.getMode());
        return out;
    }
}
