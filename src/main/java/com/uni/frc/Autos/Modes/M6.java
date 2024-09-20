package com.uni.frc.Autos.Modes;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.Autos.AutoBase;
import com.uni.frc.Autos.Actions.LambdaAction;
import com.uni.frc.Autos.Actions.ParallelAction;
import com.uni.frc.Autos.Actions.SeriesAction;
import com.uni.frc.Autos.Actions.TrajectoryAction;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.List;

public class M6 extends AutoBase {
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = 1;
    PathPlannerPath path = PathPlannerPath.fromPathFile("StartToClose1");
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("close1 to close 2");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("close1 to close 2");
    
    PathPlannerTrajectory trajectory = addTrajectory(path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation)));
    PathPlannerTrajectory trajectory1 = addTrajectory(path1.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation)));
    PathPlannerTrajectory trajectory2 = addTrajectory(path1.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation)));

    @Override
    public void routine() {
        runAction(new ParallelAction(List.of(
            new LambdaAction(()->s.intakeState(1)),
            new SeriesAction(
            new TrajectoryAction(trajectory),
            new LambdaAction(()->s.shootState(true)))
        )
        ));
        runAction(new ParallelAction(List.of(
            new LambdaAction(()->s.intakeState(1)),
            new SeriesAction(
            new TrajectoryAction(trajectory1),
            new LambdaAction(()->s.shootState(true)))
        )
        ));
        runAction(new ParallelAction(List.of(
            new LambdaAction(()->s.intakeState(1)),
            new SeriesAction(
            new TrajectoryAction(trajectory2),
            new LambdaAction(()->s.shootState(true)))
        )
        ));
 
    }}