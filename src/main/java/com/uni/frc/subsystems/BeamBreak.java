package com.uni.frc.subsystems;


import com.uni.frc.Ports;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;


public class BeamBreak {
    DigitalInput sensors;
    BeamBreak(int port){
        sensors = new DigitalInput(Ports.IndexerBeamBreakPort);
        }

    public boolean get(){
        return sensors.get();
    }

    


}
