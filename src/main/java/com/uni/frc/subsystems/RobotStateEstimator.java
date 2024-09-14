// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.subsystems;

import com.uni.frc.loops.ILooper;
import com.uni.frc.loops.Loop;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import com.uni.lib.geometry.Pose2d;

import com.uni.lib.geometry.Twist2d;
import com.uni.lib.swerve.SwerveOdometry;

import edu.wpi.first.wpilibj.Timer;

public class RobotStateEstimator extends Subsystem {

  static RobotStateEstimator instance = null;

  private SwerveOdometry mOdometry;
  private SwerveDrive mDrive;

  public static RobotStateEstimator getInstance() {
    if (instance == null) {
      instance = new RobotStateEstimator();
      return instance;
    }

    return instance;
  }

  /** Creates a new RobotStateEstimator. */
  public RobotStateEstimator() {
    mDrive = SwerveDrive.getInstance();
    mOdometry = new SwerveOdometry(mDrive.getKinematics(), Pose2d.identity());
  }

  public void registerEnabledLoops(ILooper mEnabledLooper) {
    mEnabledLooper.register(new Loop() {
      @Override
      public void onStart(double timestamp) {
      }

      @Override
      public void onLoop(double timestamp) {
        double timeStamp = Timer.getFPGATimestamp();
        mOdometry.updateWithSwerveModuleStates(mDrive.getRobotHeading(), mDrive.getModules(), timeStamp);
        mDrive.resetModulePose(mOdometry.getPoseMeters());
        Twist2d measuredVelocity = mOdometry.getVelocity().toTwist2d();
        Twist2d predictedVelocity = mDrive.getSetPoint().toTwist2d();
        RobotState.getInstance().addOdomObservations(timeStamp, mOdometry.getPoseMeters(), measuredVelocity,
            predictedVelocity);
      }

      @Override
      public void onStop(double timestamp) {
      }
    });

  }

  public void resetOdometry(Pose2d initialPose) {
    mOdometry.resetPosition(initialPose, mDrive.getModules());
    RobotState.getInstance().reset(Timer.getFPGATimestamp(), initialPose);

  }

  public void resetModuleOdometry(Pose2d initialPose) {
    mOdometry.resetPosition(initialPose, mDrive.getModules());
  }

  @Override
  public void outputTelemetry() {
    RobotState.getInstance().outputTelemetry();
  }

  @Override
  public void stop() {
  }

}
