// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root dir[]ectory of this project.

package com.uni.frc.subsystems.Swerve;

import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.uni.frc.Constants;
import com.uni.frc.Options;
import com.uni.frc.Ports;
import com.uni.frc.Planners.AimingPlanner;
import com.uni.frc.Planners.AutoAlignMotionPlanner;
import com.uni.frc.Planners.DriveMotionPlanner;
import com.uni.frc.Planners.TargetPiecePlanner;
import com.uni.frc.Planners.AimingPlanner.AimingRequest;
import com.uni.frc.loops.ILooper;
import com.uni.frc.loops.Loop;
import com.uni.frc.subsystems.RobotState;
import com.uni.frc.subsystems.Subsystem;
import com.uni.frc.subsystems.Requests.Request;
import com.uni.frc.subsystems.Swerve.SwerveDriveModule.ModuleStatus;
import com.uni.frc.subsystems.Vision.ObjectLimeLight;
import com.uni.frc.subsystems.Vision.OdometryLimeLight;
import com.uni.frc.subsystems.gyros.Pigeon;
import com.uni.lib.Conversions;
import com.uni.lib.HeadingController;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.geometry.Twist2d;
import com.uni.lib.motion.TrajectoryIterator;
import com.uni.lib.swerve.ChassisSpeeds;
import com.uni.lib.swerve.SwerveKinematics;
import com.uni.lib.swerve.SwerveModuleState;
import com.uni.lib.util.Util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class SwerveDrive extends Subsystem {
    public static SwerveDrive instance = null;

    public static SwerveDrive getInstance() {// if doesnt have an instance of swerve will make a new one
        if (instance == null)
            instance = new SwerveDrive();
        return instance;
    }

    SwerveDriveModule frontRightModule, frontLeftModule, rearLeftModule, rearRightModule;
    List<SwerveDriveModule> modules;

    Translation2d aimingVector = new Translation2d();
    Translation2d translationVector = new Translation2d();
    public double rotationScalar = 0;
    double speedPercent = 1;
    double rotationalVel;
    ChassisSpeeds desSpeeds;

    Pigeon gyro;
    OdometryLimeLight odometryVision;
    ObjectLimeLight objectVision;
    Pose2d drivingPose;

    Pose2d poseMeters = new Pose2d();
    Rotation2d mTrackingAngle = Rotation2d.identity();


    List<SwerveDriveModule> positionModules;
    Translation2d currentVelocity = new Translation2d();

    List<Translation2d> moduleVectors;

    final double translationDeadband = 0.1;
    final double rotationDeadband = 0.1;
    private boolean robotCentric = false;
    double desiredRotationScalar;
    double distanceTraveled;

    double lastUpdateTimestamp = 0;

    SwerveKinematics inverseKinematics = new SwerveKinematics();
    AutoAlignMotionPlanner mAutoAlignMotionPlanner;
    DriveMotionPlanner mDriveMotionPlanner = new DriveMotionPlanner();
    AimingPlanner mAimingPlanner;
    TargetPiecePlanner mTargetPiecePlanner;
    RobotState robotState;
    boolean mOverrideHeading = false;
    HeadingController headingController = new HeadingController();
boolean mOverrideTrajectory = false;

    double currentSpeed = 0;
    double bestDistance;
    boolean lob = false;
    boolean stateHasChanged = false;
    boolean modeHasChanged = false;
    double lastTimeStamp = 0;

    public SwerveDrive() {

        frontRightModule = new SwerveDriveModule(Ports.FRONT_RIGHT_ROTATION, Ports.FRONT_RIGHT_DRIVE, 0,
                Constants.kFrontRightStartingEncoderPosition, Constants.kFrontRightPosition, true,
                Constants.mFrontRightPosition);
        frontLeftModule = new SwerveDriveModule(Ports.FRONT_LEFT_ROTATION, Ports.FRONT_LEFT_DRIVE, 1,
                Constants.kFrontLeftStartingEncoderPosition, Constants.kFrontLeftPosition, true,
                Constants.mFrontLeftPosition);
        rearLeftModule = new SwerveDriveModule(Ports.REAR_LEFT_ROTATION, Ports.REAR_LEFT_DRIVE, 2,
                Constants.kRearLeftStartingEncoderPosition, Constants.kRearLeftPosition, true,
                Constants.mRearLeftPosition);
        rearRightModule = new SwerveDriveModule(Ports.REAR_RIGHT_ROTATION, Ports.REAR_RIGHT_DRIVE, 3,
                Constants.kRearRightStartingEncoderPosition, Constants.kRearRightPosition, true,
                Constants.mRearRightPosition);
        modules = Arrays.asList(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule);

        // sets which ways the modules turn
        frontRightModule.invertDriveMotor(true);
        frontLeftModule.invertDriveMotor(false);
        rearLeftModule.invertDriveMotor(false);
        rearRightModule.invertDriveMotor(true);
        positionModules = Arrays.asList(frontRightModule, frontLeftModule, rearLeftModule, rearRightModule);
        distanceTraveled = 0;

        gyro = Pigeon.getInstance();
        odometryVision = OdometryLimeLight.getInstance();
        objectVision = ObjectLimeLight.getInstance();
        robotState = RobotState.getInstance();
        mAutoAlignMotionPlanner = new AutoAlignMotionPlanner();
        mAimingPlanner = new AimingPlanner();
        mTargetPiecePlanner = new TargetPiecePlanner();

    }

    public enum State {
        MANUAL,
        TRAJECTORY,
        OFF,
        TARGETOBJECT,
        AIMING,
        ALIGNMENT,
        SNAP,
    }

    public enum TrajectoryMode {
        TRACKING,
        FOLLOWING,
    }

    private State currentState = State.MANUAL;
    private TrajectoryMode currentMode = TrajectoryMode.FOLLOWING;

    public State getState() {
        return this.currentState;
    }

    public Pose2d getPose() {
        return poseMeters;
    }

    public void setState(State desiredState) {
        if (desiredState != currentState)
            stateHasChanged = true;
        currentState = desiredState;
    }

    private ChassisSpeeds updatePathFollower(Pose2d current_pose) {
			final double now = Timer.getFPGATimestamp();
            return mDriveMotionPlanner.update(now, current_pose);
    }

    public void SetOverideHeading(boolean override) {
        mOverrideHeading = override;
    }

    public void setTrackingRotation(Rotation2d angle) {
        mTrackingAngle = angle;
    }

    public void setMode(TrajectoryMode desiredMode) {
        if (desiredMode != currentMode)
            modeHasChanged = true;
        currentMode = desiredMode;
    }

    public SwerveKinematics getKinematics() {
        return inverseKinematics;
    }

    public void resetModulePose(Pose2d newpose) {
        modules.forEach((m) -> m.resetPose(newpose));
    }

    public void setSpeedPercent(double percent) {
        speedPercent = (1 - (percent * Constants.GrannyModeWeight));
    }

    public void sendInput(double x, double y, double rotation) {
        translationVector = new Translation2d(x, y).scale(speedPercent);
        if (Math.abs(rotation) <= rotationDeadband) {
            rotation = 0;
        }
        if (rotation == 0 && rotationScalar != 0) {
            headingController.disableHeadingController(true);
        }
        rotationScalar = rotation;
        final double scaleValue = 1.5;
        double inputMagnitude = translationVector.norm();
        inputMagnitude = Math.pow(inputMagnitude, scaleValue);
        inputMagnitude = Util.deadband(translationDeadband, inputMagnitude);
        if (translationVector.norm() <= translationDeadband) {
            translationVector = new Translation2d();
        }
        if (DriverStation.getAlliance().get().equals(Alliance.Blue))
            translationVector = translationVector.inverse();
        rotationScalar *= 0.025;
    }

    public void setAlignment(Pose2d pose) {
        mAutoAlignMotionPlanner.setTargetPoint(pose);
        robotState.setDisplaySetpointPose(pose);
    }

    public void setLob(boolean newlob) {
        this.lob = newlob;
    }

    public void commandModules(List<Translation2d> moduleVectors) {
        this.moduleVectors = moduleVectors;
        for (int i = 0; i < moduleVectors.size(); i++) {
            if (Util.shouldReverse(moduleVectors.get(i).direction(),
                    Rotation2d.fromDegrees(modules.get(i).getModuleAngle()))) {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees() + 180);
                modules.get(i).setDriveOpenLoop(-moduleVectors.get(i).norm());
            } else {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees());
                modules.get(i).setDriveOpenLoop(moduleVectors.get(i).norm());

            }
        }
    }

    public void commandModulesVelocity(List<Translation2d> moduleVectors) {
        this.moduleVectors = moduleVectors;
        for (int i = 0; i < moduleVectors.size(); i++) {
            if (Util.shouldReverse(moduleVectors.get(i).direction(),
                    Rotation2d.fromDegrees(modules.get(i).getModuleAngle()))) {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees() + 180);
                modules.get(i).setDriveVelocity(-moduleVectors.get(i).norm());
            } else {
                modules.get(i).setModuleAngle(moduleVectors.get(i).direction().getDegrees());
                modules.get(i).setDriveVelocity(moduleVectors.get(i).norm());

            }
        }
    }


    	private List<Translation2d> updatePathFollowingSetpoint(ChassisSpeeds des_chassis_speeds) {

		Pose2d robot_pose_vel = new Pose2d(
				des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt * 4.0,
				des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt * 4.0,
				Rotation2d.fromRadians(
						des_chassis_speeds.omegaRadiansPerSecond * Constants.kLooperDt * 4.0));
		Twist2d twist_vel = Pose2d.log(robot_pose_vel).scaled(1.0 / (4.0 * Constants.kLooperDt));

		// ChassisSpeeds wanted_speeds;
		// if (mOverrideHeading) {
		// 	headingController.setTargetHeading(mTrackingAngle.inverse());
		// 	double new_omega = headingController.getRotationCorrection(getRobotHeading().inverse().flip(), Timer.getFPGATimestamp());
		// 	ChassisSpeeds speeds = new ChassisSpeeds(twist_vel.dx, twist_vel.dy, new_omega);
		// 	wanted_speeds = speeds;
		// } else {
		// 	wanted_speeds = new ChassisSpeeds(twist_vel.dx, twist_vel.dy, twist_vel.dtheta);
		// }
        return inverseKinematics.updateDriveVectorsVelocity(new Translation2d(twist_vel.dx, twist_vel.dy), twist_vel.dtheta, Pose2d.fromRotation(getRobotHeading()), false);
	}

   

    

    public void commandModuleDrivePowers(double power) {
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).setDriveOpenLoop(power);
        }
    }

    public void resetPose(Pose2d newPose) {
        modules.forEach((m) -> m.resetPose(newPose));
    }

    public void zeroModules() {
        modules.forEach((m) -> {
            m.resetModulePositionToAbsolute();
        });
    }

    public Rotation2d getRobotHeading() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public List<SwerveDriveModule> getModules() {
        return modules;
    }

    public boolean isStatusOK() {
        for (SwerveDriveModule m : modules) {
            if (m.getModuleStatus().equals(ModuleStatus.OK))
                return false;
        }
        return true;
    }
        public void setTrajectory(TrajectoryIterator trajectory){
        mOverrideTrajectory = false;
        mDriveMotionPlanner.reset();
        mDriveMotionPlanner.setTrajectory(trajectory);
        setState(State.TRAJECTORY);

    }


    public void overrideTrajectory(boolean newVal){
        mOverrideTrajectory = newVal;
    }
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {

            }

            @Override
            public void onLoop(double timestamp) {
                outputTelemetry();
                poseMeters = robotState.getKalmanPose(timestamp);
                drivingPose = Pose2d.fromRotation(getRobotHeading());
                double rotationCorrection;
                switch (currentState) {
                    case MANUAL:
                        if (stateHasChanged) {
                            headingController.setTargetHeading(getRobotHeading().inverse().flip());
                        }
                        rotationCorrection = headingController.updateRotationCorrection(
                                drivingPose.getRotation().inverse().flip(),
                                timestamp);
                        if (translationVector.norm() == 0 || rotationScalar != 0) {
                            rotationCorrection = 0;
                        }
                        commandModules(inverseKinematics.updateDriveVectors(translationVector,
                                rotationScalar, drivingPose, robotCentric));
                        break;

                    case ALIGNMENT:
                        // if (stateHasChanged) {
                        //     mAutoAlignMotionPlanner.reset();
                        // }
                        // ChassisSpeeds targetChassisSpeeds = updateAutoAlign();
                        // commandModuleVelocitys(inverseKinematics.updateDriveVectors(new Translation2d(
                        //         -targetChassisSpeeds.vxMetersPerSecond,
                        //         targetChassisSpeeds.vyMetersPerSecond),
                        //         targetChassisSpeeds.omegaRadiansPerSecond * 8,
                        //         poseMeters,
                        //         false));
                        break;

                    case TRAJECTORY:
                        if (DriverStation.isTeleop() && translationVector.norm() != 0) {
                            setState(State.MANUAL);
                        }
                        ChassisSpeeds desPathSpeed = updatePathFollower(poseMeters);
                        List<Translation2d> real_module_setpoints = updatePathFollowingSetpoint(desPathSpeed);
                        commandModulesVelocity(real_module_setpoints);
                       break;
                    case TARGETOBJECT:
                        rotationCorrection = mTargetPiecePlanner.updateAiming(timestamp,
                                objectVision.getLatestVisionUpdate(), headingController, getRobotHeading());
                        commandModules(
                                inverseKinematics.updateDriveVectors(translationVector.scale(.5),rotationScalar, drivingPose, robotCentric));
                        break;
                    case AIMING:
                        Pose2d demandedAngle;
                        if (!lob)
                            demandedAngle = mAimingPlanner.updateAiming(
                                    timestamp,
                                    RobotState.getInstance().getLatestPoseFromOdom().getValue(),
                                    Pose2d.fromTranslation(RobotState.getInstance().getLatestVisionPoseComponent()),
                                    AimingRequest.SPEAKER,
                                    odometryVision.getLatestVisionUpdate(),
                                    headingController,
                                    robotState.getSmoothedVelocity());
                        else
                            demandedAngle = mAimingPlanner.updateAiming(
                                    timestamp,
                                    RobotState.getInstance().getLatestPoseFromOdom().getValue(),
                                    Pose2d.fromTranslation(RobotState.getInstance().getLatestVisionPoseComponent()),
                                    AimingRequest.LOB,
                                    odometryVision.getLatestVisionUpdate(),
                                    headingController,
                                    robotState.getSmoothedVelocity());
                        commandModules(
                                inverseKinematics.updateDriveVectors(translationVector.scale(.3),
                                        demandedAngle.getRotation().getDegrees(), drivingPose,
                                        robotCentric));
                        break;

                    case OFF:
                        commandModules(inverseKinematics.updateDriveVectors(new Translation2d(), 0, drivingPose,
                                robotCentric));
                        break;
                }
                stateHasChanged = false;

            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    public ChassisSpeeds updateAutoAlign() {
        final double now = Timer.getFPGATimestamp();
        var fieldToOdometry = robotState.getAbsoluteVisionPoseComponent(now);
        var odomToVehicle = robotState.getPoseFromOdom(now);
        ChassisSpeeds output = mAutoAlignMotionPlanner.updateAutoAlign(now, odomToVehicle,
                Pose2d.fromTranslation(fieldToOdometry), robotState.getMeasuredVelocity(), headingController,
                getRobotHeading());
        return output;
    }

    public void snap(double r) {
        setState(State.SNAP);
        headingController.setTargetHeading(Rotation2d.fromDegrees(r));
    }

    public void fieldZeroSwerve() {// starts the zero 180 off
        headingController.setTargetHeading(Rotation2d.fromDegrees(0));
        gyro.setAngle(180);
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            headingController.setTargetHeading(Rotation2d.fromDegrees(180));
            gyro.setAngle(0);
        }

    }

    @Override
    public void readPeriodicInputs() {
        modules.forEach((m) -> {
            m.readPeriodicInputs();
        });
    }

    @Override
    public void writePeriodicOutputs() {
        modules.forEach((m) -> {
            m.writePeriodicOutputs();
        });
    }

    public void resetGryo(double angle) {
        headingController.setTargetHeading(Rotation2d.fromDegrees(angle));
        gyro.setAngle(angle);
    }

    public void zeroSwerve() {// zeros gyro
        headingController.setTargetHeading(Rotation2d.fromDegrees(0));
        gyro.setAngle(0);
        resetPose(new Pose2d(new Translation2d(1.9, 4.52), Rotation2d.fromDegrees(0)));
    }
    public boolean isTrajectoryFinished() {
        Logger.recordOutput("Trajectory Error", mDriveMotionPlanner.getEndPosition().getTranslation().translateBy(poseMeters.getTranslation().inverse()).norm());
         return mDriveMotionPlanner.getEndPosition().getTranslation().translateBy(poseMeters.getTranslation().inverse()).norm() < Units.inchesToMeters(4);
    }

    
    public void resetEncoders() {// zeros encoders
        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).resetEncoders();
        }

    }

    public Request setStateRequest(State state) {
        return new Request() {
            @Override
            public void act() {
                setState(state);
            }
        };
    }

    public Request setModeRequest(TrajectoryMode mode) {
        return new Request() {
            @Override
            public void act() {
                setMode(mode);
            }
        };
    }

    public Request isAimedRequest() {
        return new Request() {
            @Override
            public boolean isFinished() {
                return mAimingPlanner.isAimed();
            }
        };
    }

    public Request isAlignedRequest() {
        return new Request() {
            @Override
            public boolean isFinished() {
                return mAutoAlignMotionPlanner.getAutoAlignmentCompleted();
            }
        };
    }

    public synchronized ChassisSpeeds getSetPoint() {
        var desiredThrottleSpeed = translationVector.x() * Constants.SwerveMaxspeedMPS;
        var desiredStrafeSpeed = translationVector.y() * Constants.SwerveMaxspeedMPS;
        var desiredRotationSpeed = rotationScalar * Conversions.falconToMPS(Constants.kSwerveRotationMaxSpeed,
                Constants.kWheelCircumference, Options.driveRatio);
        return ChassisSpeeds.fromRobotRelativeSpeeds(desiredThrottleSpeed, desiredStrafeSpeed, desiredRotationSpeed);
    }

    public void snapToPoint(Pose2d targetPoint) {
        if (mAutoAlignMotionPlanner != null) {
            if (currentState != State.ALIGNMENT) {
                mAutoAlignMotionPlanner.reset();
                setState(State.ALIGNMENT);
            }
        }
        mAutoAlignMotionPlanner.setTargetPoint(targetPoint);
        robotState.setDisplaySetpointPose(targetPoint);
    }

    @Override
    public void outputTelemetry() {
        Logger.recordOutput("Swerve/State", getState());
        modules.forEach((m) -> m.outputTelemetry());
        Logger.recordOutput("Swerve/Heading", getRobotHeading().getDegrees());
        Logger.recordOutput("Swerve/DesiredHeading", headingController.getTargetHeading());
    }

    @Override
    public void stop() {// stops everything
        setState(State.MANUAL);
        translationVector = new Translation2d();
        rotationScalar = 0;

        commandModuleDrivePowers(0);

    }

}