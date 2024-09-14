// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;


public class Music extends Subsystem{
  public static Music instance = null;

    public static Music getInstance() {
        if (instance == null)
            instance = new Music();
        return instance;
    }
  PeriodicIO mPeriodicIO;
  Orchestra orchestra = new Orchestra();
  
  public Music() {
  mPeriodicIO = new PeriodicIO();

    orchestra = new Orchestra();

  }
  public void play(String filepath){
    orchestra.loadMusic(filepath);
    if(mPeriodicIO.isPlaying){
      mPeriodicIO.wasPlaying = true;
    }else{
      mPeriodicIO.wasPlaying = false;
    }
    mPeriodicIO.isPlaying = true;
  }
  public void stopMusic(){

    mPeriodicIO.wasPlaying = false;
    mPeriodicIO.isPlaying = false;
  }
  public void addInstrument(TalonFX instrument){
    orchestra.addInstrument(instrument);
  }
  @Override
  public void readPeriodicInputs(){
    if(mPeriodicIO.isPlaying && !mPeriodicIO.wasPlaying)
      orchestra.play();
    
  }

  @Override
  public void outputTelemetry() {
  
  }
  @Override
  public void stop() {
    stopMusic();
  }
  public static class PeriodicIO{
    boolean isPlaying = false;
    boolean wasPlaying = false;

  }

}
