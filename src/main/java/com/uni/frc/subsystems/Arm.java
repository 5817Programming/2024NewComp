// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.uni.frc.Constants;
import com.uni.frc.Ports;
import com.uni.frc.subsystems.Requests.Request;
import com.uni.lib.TalonConfigs;

public class Arm extends Subsystem {
  
  private PeriodicIO mPeriodicIO = new PeriodicIO();
  private TalonFX armMotor = new TalonFX(Ports.Arm,"Minivore");
  private TalonFXConfiguration pivotConfig = TalonConfigs.armConfigs();

  
  public static Arm instance = null;

  public static Arm getInstance() {
    if (instance == null)
      instance = new Arm();
    return instance;
  }

  public Arm() {
    configMotors();
  }

  public enum ControlMode {
    MotionMagic,
    Percent,
  }

  public enum State {
    AMP(Constants.ArmConstants.AMP),
    MAXDOWN(Constants.ArmConstants.MAX_DOWN), 
    PARTIAL(Constants.ArmConstants.PARTIAL);

    double output = 0;

    State(double output){
      this.output = output;
    }
  }

  public void setRamp(double rampTime) {
    armMotor.getConfigurator().refresh(pivotConfig);
    pivotConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = rampTime;
    armMotor.getConfigurator().apply(pivotConfig);
  }

  public void configMotors() {
    pivotConfig = TalonConfigs.swerveDriveConfig();
    armMotor.getConfigurator().apply(pivotConfig);
  }

  public void setMotionMagic(double position){
    mPeriodicIO.rotationControlMode = ControlMode.MotionMagic;

    mPeriodicIO.rotationDemand = position;
  }

  public void setPivotPercent(double percentage) {
    mPeriodicIO.rotationControlMode = ControlMode.Percent;
    mPeriodicIO.rotationDemand = percentage;
  }



  public boolean atTarget(){
    return Math.abs(-mPeriodicIO.rotationDemand - mPeriodicIO.rotationPosition) < 1;
  }

  public void conformToState(State state) {
    setMotionMagic(state.output);
  }
  public void conformToState(double Override) {
    setMotionMagic(Override);
  }

  public void motionMagic(){
    armMotor.setControl(new MotionMagicVoltage(-mPeriodicIO.rotationDemand));
  }

  public void setPercent(){
    armMotor.setControl(new DutyCycleOut(-mPeriodicIO.rotationDemand, true, false, false, false));
     
  }

  public Request stateRequest(State state) {
    return new Request() {

      @Override
      public void act() {
        conformToState(state);
      }
    };
  }

  public Request stateRequest(Double position) {
    return new Request() {

      @Override
      public void act() {
        conformToState(position);
      }
    };
  }

  public Request setpivotPercentRequest(double percentage) {
    return new Request() {

      @Override
      public void act() {
        setPivotPercent(percentage);
      }

    };

  }

  public Request atTargetRequest(){
    return new Request() {
      @Override
        public boolean isFinished() {
            return  atTarget();
        }
    };
  }

  public double getStatorCurrent() {
    return mPeriodicIO.statorCurrent;
  }

  @Override
  public void writePeriodicOutputs() {
    mPeriodicIO.rotationPosition = armMotor.getPosition().getValueAsDouble();
    mPeriodicIO.velocity = armMotor.getVelocity().getValueAsDouble();
    mPeriodicIO.statorCurrent = armMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void readPeriodicInputs() {
    switch (mPeriodicIO.rotationControlMode) {
      case MotionMagic:
        motionMagic();
        break;
      case Percent:
        setPercent();
        break;
    }
  }

  @Override
  public void outputTelemetry() {
  }

  @Override
  public void stop() {
    setpivotPercentRequest(0);
  }

  public static class PeriodicIO {
    ControlMode rotationControlMode = ControlMode.MotionMagic;
    double rotationPosition = 0;
    double velocity = 0;
    double statorCurrent = 0;

    double rotationDemand = 0.0;
  }
}
