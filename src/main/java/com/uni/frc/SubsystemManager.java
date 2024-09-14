// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc;

import java.util.List;

import com.uni.frc.subsystems.Subsystem;

/** Add your docs here. */
public class SubsystemManager {//up   dates all new subsystem methods

    public static List<Subsystem> allSubsystems;


    public SubsystemManager() {

    }

    public void addSystems(List<Subsystem> subsystemList) {
        allSubsystems = subsystemList;
    }

    public void readSystemsPeriodicInputs() {
        allSubsystems.forEach((system)-> {system.readPeriodicInputs();});
    }
    public void writeSubsystemsPeriodicOutputs() {
        allSubsystems.forEach((system)->{system.writePeriodicOutputs();});
    }
    public void updateSubsystems() {
        allSubsystems.forEach((system)-> {system.update();});
    }
    public void outputSystemsTelemetry() {
        allSubsystems.forEach((system)-> {system.outputTelemetry();});
    }
    public void stopSubsystems() {
        allSubsystems.forEach((system)-> {system.stop();});
    }
}
