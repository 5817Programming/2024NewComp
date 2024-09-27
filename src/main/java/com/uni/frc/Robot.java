// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc;

import java.util.Arrays;
import java.util.HashMap;

import org.littletonrobotics.junction.LoggedRobot;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.uni.frc.Autos.AutoBase;
import com.uni.frc.Autos.AutoExecuter;
import com.uni.frc.Autos.Modes.M6;
import com.uni.frc.Controls.Controls;
import com.uni.frc.loops.Looper;
import com.uni.frc.subsystems.Arm;
import com.uni.frc.subsystems.Climb;
import com.uni.frc.subsystems.Indexer;
import com.uni.frc.subsystems.Intake;
import com.uni.frc.subsystems.Lights;
import com.uni.frc.subsystems.Music;
import com.uni.frc.subsystems.Pivot;
import com.uni.frc.subsystems.RobotState;
import com.uni.frc.subsystems.RobotStateEstimator;
import com.uni.frc.subsystems.Shooter;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.SuperStructure.SuperState;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import com.uni.frc.subsystems.Vision.ObjectLimeLight;
import com.uni.frc.subsystems.Vision.OdometryLimeLight;
import com.uni.frc.subsystems.gyros.Gyro;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;


public class Robot extends LoggedRobot {

    Controls controls;
    SubsystemManager mSubsystemManager;
    SuperStructure s = SuperStructure.getInstance();
  AutoExecuter autoExecuter = new AutoExecuter();
    SwerveDrive swerve;
    double yaw;
    OdometryLimeLight vision;
    Gyro pigeon;
    AutoBase auto = new M6();
    public LoggedDashboardChooser<AutoBase> autoChooser = new LoggedDashboardChooser<>("AutoChooser");
    private final Looper mEnabledLooper = new Looper();
  
  HashMap<String,AutoBase> autos = new HashMap<String,AutoBase>();
    @Override
    public void robotInit() {
      autos.put("Middle 6", new M6());
  
      // autos.put("1", new Shoot());
  
      DriverStation.startDataLog(DataLogManager.getLog());
  
      RobotState.getInstance().resetKalmanFilters(Timer.getFPGATimestamp());
      for(HashMap.Entry<String, AutoBase> entry : autos.entrySet()) {
        String N = entry.getKey();
        AutoBase A = entry.getValue();
        autoChooser.addOption(N, A);
      }
  
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
      swerve = SwerveDrive.getInstance();
      controls = Controls.getInstance();
      vision = OdometryLimeLight.getInstance();
      swerve.zeroModules();
      mSubsystemManager = SubsystemManager.getInstance();
  
      mSubsystemManager.setSubsystems(
          SwerveDrive.getInstance(),
          SuperStructure.getInstance(), 
          OdometryLimeLight.getInstance(),
          RobotStateEstimator.getInstance(),
          ObjectLimeLight.getInstance(),
          Shooter.getInstance(),
          Indexer.getInstance(),
          Intake.getInstance(),
          Pivot.getInstance(),
          Climb.getInstance(),
          Lights.getInstance(),
          Arm.getInstance()
          );
          mSubsystemManager.registerEnabledLoops(mEnabledLooper);
      }
  
    @Override
    public void robotPeriodic() {
      auto = autoChooser.get();
      
      mEnabledLooper.outputToSmartDashboard();
          mSubsystemManager.outputLoopTimes();
      SubsystemManager.getInstance().outputTelemetry();
      OdometryLimeLight.getInstance().readInputsAndAddVisionUpdate();
    }
  
  
  
    @Override
    public void autonomousInit() {
      swerve = SwerveDrive.getInstance();
      swerve.resetGryo(OdometryLimeLight.getInstance().getMovingAverageHeading());
      swerve.zeroModules();
      SuperStructure.getInstance().setState(SuperState.AUTO);
      mEnabledLooper.start();
      autoExecuter.setAuto(new M6());
      autoExecuter.start();
    }
  
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() { 
    }
  
    /** This function is called once when teleop is enabled. */  
    @Override
    public void teleopInit() {
      mEnabledLooper.start();
      swerve = SwerveDrive.getInstance();
      RobotStateEstimator.getInstance().resetOdometry(new Pose2d(15.18,5.48,Rotation2d.kIdentity));
      // swerve.fieldzeroSwerve();
      swerve.zeroModules();
  
    }
  
    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
      controls.update();
    }
  
    /** This function is called once when the robot is disabled. */
  
    @Override
    public void disabledInit() {
      OdometryLimeLight.getInstance().resetMovingAverageHeading();
      mSubsystemManager.stop();
      SuperStructure.getInstance().clearQueues();
      mEnabledLooper.stop();
      autoExecuter.stop();
      autoExecuter = new AutoExecuter();
    }
  
    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
      RobotState.getInstance().outputTelemetry();
    }
  
    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }
  
    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
      
  
    }
  }
  