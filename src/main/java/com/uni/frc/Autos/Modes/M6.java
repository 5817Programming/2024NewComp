package com.uni.frc.Autos.Modes;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.Autos.AutoBase;
import com.uni.frc.Autos.Actions.LambdaAction;
import com.uni.frc.Autos.Actions.ParallelAction;
import com.uni.frc.Autos.Actions.SeriesAction;
import com.uni.frc.Autos.Actions.TrajectoryAction;
import com.uni.frc.Autos.Actions.WaitAction;
import com.uni.frc.Autos.Actions.WaitForSuperstructureAction;
import com.uni.frc.subsystems.Pivot;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.util.List;

import org.opencv.features2d.FastFeatureDetector;

public class M6 extends AutoBase {
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = 1;
    PathPlannerPath path = PathPlannerPath.fromPathFile("StartToClose1");
    PathPlannerPath path1 = PathPlannerPath.fromPathFile("Close1ToClose2");
    PathPlannerPath path2 = PathPlannerPath.fromPathFile("Close2ToClose3");
    PathPlannerPath path3 = PathPlannerPath.fromPathFile("Close3ToFar1");
    PathPlannerPath path4 = PathPlannerPath.fromPathFile("Far1ToShot1");
    

    PathPlannerTrajectory trajectory = addTrajectory(path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation)));
    PathPlannerTrajectory trajectory1 = addTrajectory(path1.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation)));
    PathPlannerTrajectory trajectory2 = addTrajectory(path2.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation)));
    PathPlannerTrajectory trajectory3 = addTrajectory(path3.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation)));
    PathPlannerTrajectory trajectory4 = addTrajectory(path4.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation)));


    @Override
    public void routine() {
        Pivot.getInstance().conformToState(Pivot.State.MAXUP);
        runAction(new WaitAction(1));
        runAction(new LambdaAction(() -> s.shootState(false)));
        runAction(new WaitAction(1));
        runAction(new ParallelAction(List.of(
            new TrajectoryAction(trajectory),
            new LambdaAction(() -> s.intakeState(2))
            )));
        runAction(new WaitAction(1));
        runAction(new LambdaAction(() -> s.shootState(false)));
        runAction(new WaitForSuperstructureAction(1));
        runAction(new WaitAction(.5));
        runAction(new ParallelAction(List.of(
            new TrajectoryAction(trajectory1),
            new LambdaAction(() -> s.intakeState(2))
            )));
        runAction(new WaitAction(1));
        runAction(new LambdaAction(() -> s.shootState(false)));
        runAction(new WaitForSuperstructureAction(1));
        runAction(new WaitAction(.5));
        runAction(new ParallelAction(List.of(
            new TrajectoryAction(trajectory2),
            new LambdaAction(() -> s.intakeSta))
            )));
        runAction(new WaitAction(1));
        runAction(new LambdaAction(() -> s.shootState(false)));
        runAction(new WaitForSuperstructureAction(1));
       // runAction(new ParallelAction(List.of(
        //     new LambdaAction(()->s.intakeState(1)),
        //     new SeriesAction(
        //     new TrajectoryAction(trajectory1),
        //     new LambdaAction(()->s.shootState(true)))
        // )
        // ));
        // runAction(new ParallelAction(List.of(
        //     new LambdaAction(()->s.intakeState(1)),
        //     new SeriesAction(
        //     new TrajectoryAction(trajectory2),
        //     new LambdaAction(()->s.shootState(true)))
        // )
        // ));
 
    } 
}