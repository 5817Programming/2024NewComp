package com.uni.lib.motion;


import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import org.littletonrobotics.junction.Logger;

/** Custom PathPlanner version of SwerveControllerCommand */
public class PathStateGenerator{
  private final Timer timer = new Timer();
  public static PathStateGenerator instance = null;
  private PathPlannerTrajectory transformedTrajectory;

  double desiredRotation = 0;
  double speed = 1;
  Pose2d currentPose;
  boolean useEvents = false;
  boolean ran = false;

  int EventIndex = 0;

  public PathStateGenerator() {
  }

  public static PathStateGenerator getInstance() {// if doesnt have an instance of swerve will make a new one
      if (instance == null)
          instance = new PathStateGenerator();
      return instance;
  }


  public void startTimer() {
    this.timer.start();
  }

  public void setTrajectory(PathPlannerTrajectory trajectory) {
    resetTimer();
    
    this.transformedTrajectory = trajectory;
  }

  public Pose2d sample(double timestamp){
    return new Pose2d(transformedTrajectory.sample(timestamp).getTargetHolonomicPose());
  }

  public Pose2d getDesiredPose2d(boolean useAllianceColor) {
    return getDesiredPose2d(useAllianceColor,timer.get());
  }
  public Pose2d getDesiredPose2d(boolean useAllianceColor,double time) {
    double currentTime = this.timer.get();
    State desiredState = transformedTrajectory.sample(currentTime);
    double desiredRotation = -((State) transformedTrajectory.sample(currentTime)).targetHolonomicRotation
        .getDegrees();
    double desiredX = desiredState.getTargetHolonomicPose().getTranslation().getX();
    double desiredY = desiredState.getTargetHolonomicPose().getTranslation().getY();
    Logger.recordOutput("desiredPose", new Pose2d(desiredX, desiredY, Rotation2d.fromDegrees(desiredRotation)).toWPI());
    if (alliance()&& useAllianceColor)
      return new Pose2d(new Translation2d(reflect(desiredX), desiredY), Rotation2d.fromDegrees(desiredRotation));
    else
      return new Pose2d(new Translation2d(desiredX, desiredY), Rotation2d.fromDegrees(desiredRotation).flip().inverse());

  }

  public double reflect(double x) {
    return 16.5-x;
  }

  public boolean alliance() {
    return DriverStation.getAlliance().get().equals(Alliance.Red);
  }

  public Pose2d getInitial(PathPlannerTrajectory trajectory,double Rotation, boolean useAllianceColor) {
    double initX = trajectory.getInitialState().positionMeters.getX();
    double initY = trajectory.getInitialState().positionMeters.getY();
    double initRot = Rotation;
    if (alliance() && useAllianceColor) {
      return new Pose2d(new Translation2d(reflect(initX), initY),
          Rotation2d.fromDegrees(initRot).flip());
    } else {
      return new Pose2d(new Translation2d(initX, initY),
         Rotation2d.fromDegrees(initRot));
    }
  }

  public void stopTimer(){
    timer.stop();
  }

  public double getrotation() {
    return desiredRotation;
  }

  public void resetTimer() {
    timer.reset();
    timer.stop();
  }

  public boolean isFinished() {

    return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds()+ .2);

  }


  // public double percentageDone() {
  //   return (this.timer.get() / (transformedTrajectory.getTotalTimeSeconds() + .2));
  // }

  public double getTime(){
    return this.timer.get();
  }

}
