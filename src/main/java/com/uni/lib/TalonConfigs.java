// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.uni.frc.Constants;
import com.uni.frc.Ports;

/** Add your docs here. */
public class TalonConfigs {

    public static TalonFXConfiguration swerveRotationConfig(){
        TalonFXConfiguration motionMagicConfig = new TalonFXConfiguration();

        motionMagicConfig.Slot0.kP = 6;
        motionMagicConfig.Slot0.kS = 0;
        motionMagicConfig.Slot0.kV = 0;
        motionMagicConfig.MotionMagic.MotionMagicCruiseVelocity = 98;
        motionMagicConfig.MotionMagic.MotionMagicAcceleration = 1000;
        motionMagicConfig.CurrentLimits.StatorCurrentLimit = 80;


        return motionMagicConfig;
    }

    public static TalonFXConfiguration swerveDriveConfig() {
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.Slot0.kV = .12;
        driveConfigs.Slot0.kS = .25;
        driveConfigs.Slot0.kA = .01;
        driveConfigs.Slot0.kP = .1;
        driveConfigs.Slot0.kI = 0;
        driveConfigs.Slot0.kD = 0;
        driveConfigs.CurrentLimits.StatorCurrentLimit = 80;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = 45;
        driveConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .2;
        driveConfigs.MotionMagic.MotionMagicCruiseVelocity = 80;
        driveConfigs.MotionMagic.MotionMagicAcceleration = 240;

      return driveConfigs;
    }


    public static TalonFXConfiguration pivotConfigs() {
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.Slot0.kV = -20;
        driveConfigs.Slot0.kS = 0;
        driveConfigs.Slot0.kA = 0;
        driveConfigs.Slot0.kP = -12;
        driveConfigs.Slot0.kI = 0;
        driveConfigs.Slot0.kD = 0;
        driveConfigs.Slot0.kG = -0.05;
        driveConfigs.Feedback.FeedbackRemoteSensorID = Ports.PivotEncoder;
        driveConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        

        driveConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =.5; 
        driveConfigs.CurrentLimits.StatorCurrentLimit = 80;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = 45;

        driveConfigs.MotionMagic.MotionMagicCruiseVelocity = 1.5;
        driveConfigs.MotionMagic.MotionMagicAcceleration = 0.75;

      return driveConfigs;

    }
    public static TalonFXConfiguration elevatorConfigs() {
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.Slot0.kV = 0;
        driveConfigs.Slot0.kS = 0;
        driveConfigs.Slot0.kA = 0;
        driveConfigs.Slot0.kP = 3;
        driveConfigs.Slot0.kI = 0;
        driveConfigs.Slot0.kD = 0;
        driveConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =.5; 
        driveConfigs.CurrentLimits.StatorCurrentLimit = 80;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = 45;

        driveConfigs.MotionMagic.MotionMagicCruiseVelocity = ((int) (Constants.kSwerveDriveMaxSpeed * 0.9));
        driveConfigs.MotionMagic.MotionMagicAcceleration = ((int) (Constants.kSwerveDriveMaxSpeed));

      return driveConfigs;
    }

    public static TalonFXConfiguration armConfigs() {
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.Slot0.kV = 2;
        driveConfigs.Slot0.kS = .25;
        driveConfigs.Slot0.kA = .01;
        driveConfigs.Slot0.kP = .11;
        driveConfigs.Slot0.kI = 0;
        driveConfigs.Slot0.kD = 0;
        driveConfigs.CurrentLimits.StatorCurrentLimit = 80;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = 45;
        driveConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =.5; 

        driveConfigs.MotionMagic.MotionMagicCruiseVelocity = 2;
        driveConfigs.MotionMagic.MotionMagicAcceleration = 2;

      return driveConfigs;
    }

    public static TalonFXConfiguration handConfigs() {
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.Slot0.kV = .12;
        driveConfigs.Slot0.kS = .25;
        driveConfigs.Slot0.kA = .01;
        driveConfigs.Slot0.kP = .11;
        driveConfigs.Slot0.kI = 0;
        driveConfigs.Slot0.kD = 0;
        driveConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =.5; 
        driveConfigs.CurrentLimits.StatorCurrentLimit = 80;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = 45;

        driveConfigs.MotionMagic.MotionMagicCruiseVelocity = ((int) (Constants.kSwerveDriveMaxSpeed * 0.9));
        driveConfigs.MotionMagic.MotionMagicAcceleration = ((int) (Constants.kSwerveDriveMaxSpeed));

      return driveConfigs;
    }
    public static TalonFXConfiguration intakeConfigs() {
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.Slot0.kV = .12;
        driveConfigs.Slot0.kS = .25;
        driveConfigs.Slot0.kA = .01;
        driveConfigs.Slot0.kP = .11;
        driveConfigs.Slot0.kI = 0;
        driveConfigs.Slot0.kD = 0;
        driveConfigs.CurrentLimits.StatorCurrentLimit = 80;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = 45;

        driveConfigs.MotionMagic.MotionMagicCruiseVelocity = ((int) (Constants.kSwerveDriveMaxSpeed * 0.9));
        driveConfigs.MotionMagic.MotionMagicAcceleration = ((int) (Constants.kSwerveDriveMaxSpeed));

      return driveConfigs;
    }
 public static TalonFXConfiguration indexerConfigs() {
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.Slot0.kV = .12;
        driveConfigs.Slot0.kS = .25;
        driveConfigs.Slot0.kA = .01;
        driveConfigs.Slot0.kP = .11;
        driveConfigs.Slot0.kI = 0;
        driveConfigs.Slot0.kD = 0;
        driveConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .02;
        driveConfigs.CurrentLimits.StatorCurrentLimit = 80;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = 45;

        driveConfigs.MotionMagic.MotionMagicCruiseVelocity = ((int) (Constants.kSwerveDriveMaxSpeed * 0.9));
        driveConfigs.MotionMagic.MotionMagicAcceleration = ((int) (Constants.kSwerveDriveMaxSpeed));

      return driveConfigs;
    }
  public static TalonFXConfiguration shooterConfigs() {
        TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
        driveConfigs.Slot0.kV = 0.01;
        driveConfigs.Slot0.kS = 0;
        driveConfigs.Slot0.kA = 0;
        driveConfigs.Slot0.kP = 0.04;
        driveConfigs.Slot0.kI = 0;
        driveConfigs.Slot0.kD = 0;
        driveConfigs.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .02;
        driveConfigs.CurrentLimits.StatorCurrentLimit = 80;
        driveConfigs.CurrentLimits.SupplyCurrentLimit = 45;

        driveConfigs.MotionMagic.MotionMagicCruiseVelocity = ((int) (Constants.kSwerveDriveMaxSpeed * 0.9));
        driveConfigs.MotionMagic.MotionMagicAcceleration = ((int) (Constants.kSwerveDriveMaxSpeed));

      return driveConfigs;
    }
}
