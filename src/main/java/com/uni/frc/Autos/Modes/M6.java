package com.uni.frc.Autos.Modes;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.Autos.AutoBase;
import com.uni.frc.Autos.Actions.LambdaAction;
import com.uni.frc.Autos.Actions.ParallelAction;
import com.uni.frc.Autos.Actions.SeriesAction;
import com.uni.frc.Autos.Actions.TrajectoryAction;
import com.uni.frc.Autos.Actions.WaitAction;
import com.uni.frc.Planners.DriveMotionPlanner;
import com.uni.frc.subsystems.Shooter;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.SuperStructure.SuperState;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.proto.System;

import java.util.List;

public class M6 extends AutoBase {
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = 1;
    PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("M6");
    
    PathPlannerTrajectory trajectory = addTrajectory(path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation)));

    @Override
    public void routine() {
        runAction(new ParallelAction(List.of(
                                        new TrajectoryAction(trajectory),
                                        new SeriesAction(
                                                            // new WaitAction(1),
                                                            // new LambdaAction(()-> s.intakeState())
                                        //                     new WaitAction(1),
                                        //                     new LambdaAction(()-> s.shootState(false))
                                        ))
        ));
    }}