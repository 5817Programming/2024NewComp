package com.uni.frc.Autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.uni.frc.subsystems.Shooter;
import com.uni.frc.subsystems.SuperStructure;
import com.uni.frc.subsystems.SuperStructure.SuperState;
import com.uni.frc.subsystems.Swerve.SwerveDrive;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class M6 extends AutoBase {
    SuperStructure s = SuperStructure.getInstance();
    SwerveDrive mSwerve = SwerveDrive.getInstance();
    double initRotation = 1;
    PathPlannerPath path = PathPlannerPath.fromPathFile("M6");
    
    PathPlannerTrajectory trajectory = path.getTrajectory(new ChassisSpeeds(), Rotation2d.fromDegrees(initRotation));

    @Override
    public void auto() {
        Shooter.getInstance().setPercent(.8);
        s.setState(SuperState.AUTO);
        Shooter.getInstance().setPercent(0.8);
        s.setContinuousShootState(false);
        s.setPivotState(26.5);
        s.waitState(0.2, false);

        s.shootState(false);
        s.setContinuousShootState(true);

        s.trajectoryState(trajectory, initRotation);
        registerTrajectoryEvents("M6");

    }}