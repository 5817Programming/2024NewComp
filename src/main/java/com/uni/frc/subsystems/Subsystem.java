// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.subsystems;


/** Add your docs here. */
public abstract class Subsystem {

    public void readPeriodicInputs() {
    }
    public void writePeriodicOutputs() {
    }
    public void update() {
        
    }
    public abstract void outputTelemetry();

    public abstract void stop();
}
