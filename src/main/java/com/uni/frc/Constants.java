// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;
import com.uni.lib.UndistortConstants;
import com.uni.lib.Vision.UndistortMap;
import com.uni.lib.Vision.UndistortMap_Limelight_B_640x480;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.geometry.HeavilyInspired.Obstacle;
import com.uni.lib.motion.MotionProfileConstraints;
import com.uni.lib.util.InterpolatingDouble;
import com.uni.lib.util.InterpolatingTreeMap;
import com.uni.lib.util.InterpolatingUndisortMap;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class Constants {
    public static final int kSlotIdx = 0;

    /**
     * Talon FX supports multiple (cascaded) PID loops. For
     * now we just want the primary one.
     */
    public static final int kPIDLoopIdx = 0;
    public static boolean isCompbot = hasMacAddress("00:80:2F:38:8F:8B");
    /**
     * set to zero to skip waiting for confirmation, set to nonzero to wait and
     * report to DS if action fails.
     */
    public static final int TIMEOUT_MILLISECONDS = 30;
    public static final int kCANTimeoutMs = 20;// The refresh rate of the periodic looper

    public static final double kRobotBaseWidth = 24.0; // The Robot Wheel Base Width
    public static final double kRobotBaseLength = 24.0;
    public static final double mRobotBaseWidth = Units.inchesToMeters(24); // The Robot Wheel Base Width
    public static final double mRobotBaseLength = Units.inchesToMeters(24);// The Robot Wheel Base Length

    public static final double kOuterWheelDriveDiameter = 4.0;

    // Kalman Filters
    public static final Matrix<N2, N1> kStateStdDevs = VecBuilder.fill(Math.pow(0.04, 1), Math.pow(0.04, 1));
    public static final Matrix<N2, N1> kLocalMeasurementStdDevs = VecBuilder.fill(Math.pow(0.01, 1), Math.pow(0.01, 1));
    public static final double kTagPoseRejectionFilter = 2.25; // Reject Vision Correction if Current Pose Farther than
                                                               // 3m from April Tag
    public static final double kNumInitialMeasurements = 15;// 100; //Number of Measurements to Let Into Filter Without
                                                            // Rejection to Initialize X_Hat

    // Sets the motor on a 2Dplane
   // The positions of the modules, relative to the robot's center
    public static final Translation2d kFrontRightPosition = new Translation2d(kRobotBaseWidth / 2,
            -kRobotBaseLength / 2);
    public static final Translation2d kFrontLeftPosition = new Translation2d(kRobotBaseWidth / 2,
            kRobotBaseLength / 2);
    public static final Translation2d kRearLeftPosition = new Translation2d(-kRobotBaseWidth / 2,
            kRobotBaseLength / 2);
    public static final Translation2d kRearRightPosition = new Translation2d(-kRobotBaseWidth / 2,
            -kRobotBaseLength / 2);
    public static final Translation2d mFrontRightPosition = new Translation2d(mRobotBaseWidth / 2,
            mRobotBaseLength / 2);
    public static final Translation2d mFrontLeftPosition = new Translation2d(mRobotBaseWidth / 2,
            -mRobotBaseLength / 2);
    public static final Translation2d mRearLeftPosition = new Translation2d(-mRobotBaseWidth / 2,
            -mRobotBaseLength / 2);
    public static final Translation2d mRearRightPosition = new Translation2d(-mRobotBaseWidth / 2,
            mRobotBaseLength / 2);

    public static final Translation2d[] mModulePositions = new Translation2d[] {
        mFrontRightPosition,
        mFrontLeftPosition,
        mRearLeftPosition,
        mRearRightPosition
    };
 
    public static final List<Translation2d> kModulePositions = Arrays.asList(kFrontRightPosition, kFrontLeftPosition,
            kRearLeftPosition, kRearRightPosition);

    public static double kSwerveRotationMaxSpeed = 12720 * 0.8;
    public static double GrannyModeWeight = .5;

    // Scrub Factors
    public static final boolean kSimulateReversedCarpet = false;
    public static final double[] kWheelScrubFactors = new double[] { 1.0, 1.0, 1.0, 1.0 };
    public static final double kXScrubFactorP = isCompbot?.95:0.85;
    public static final double kYScrubFactorP = isCompbot?.97:.875;
    public static final double kXScrubFactorN = isCompbot?.95:0.85;
    public static final double kYScrubFactorN = isCompbot?.97:.875;


    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    public static final double kSwerveMaxspeedMPS = 10;
    public static final double SwerveMaxspeedMPS = 120;
    public static final double kSwerveDriveMaxSpeed = 22000.0; // The theoretical max speed(For the Falcon 500s)
    public static final double kSwerveRotation10VoltMaxSpeed = 1350.0;

    public static final double kSwerveRotationReduction = Options.rotationRatio; // The Module to Motor Ratio(i.e,
                                                                                 // amount the rotation motor rotates,
                                                                                 // for every one rotation for the
                                                                                 // module)
    public static final double kSwerveWheelReduction = Options.driveRatio; // The Wheel to Motor Ratio(i.e, amount the
                                                                           // drive motor rotates, for every one
                                                                           // rotation for the wheel)
    public static final double kSwerveRotationEncoderResolution = 2048.0;
    public static final double kSwerveDriveEncoderResolution = 2048.0;

    public static final double kSwerveWheelDiameter = 4; // inches

    public static final double kSwerveEncUnitsPerWheelRev = kSwerveDriveEncoderResolution * 2048;
    public static final double kSwerveEncUnitsPerInch = kSwerveEncUnitsPerWheelRev / (Math.PI * kSwerveWheelDiameter);
    public static final double kWheelCircumference = Units.inchesToMeters(kSwerveWheelDiameter * Math.PI);

    public static final double kMaxVelocityMetersPerSecond = 5.05; // Calibrated 3/12 on Comp Bot
    public static final double kMaxAccelerationMetersPerSecondSquared = 4.4;

    public static final MotionProfileConstraints kPositionMotionProfileConstraints = new MotionProfileConstraints(
            0.8 * Constants.kMaxVelocityMetersPerSecond,
            0.8 * -Constants.kMaxVelocityMetersPerSecond,
            0.6 * Constants.kMaxAccelerationMetersPerSecondSquared);
    public static final MotionProfileConstraints kHeadingMotionProfileConstraints = new MotionProfileConstraints(
            0.5 * Constants.kMaxAngularSpeedRadiansPerSecond,
            0.5 * -Constants.kMaxAngularSpeedRadiansPerSecond,
            1.0 * Constants.kMaxAngularAccelerationRadiansPerSecondSquared);
    /// The absolute starting postion for each module
    public static final double kMaxAngularSpeedRadiansPerSecond = kMaxVelocityMetersPerSecond /
    Math.hypot(mRobotBaseLength / 2.0, mRobotBaseWidth / 2.0);
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = kMaxAccelerationMetersPerSecondSquared /
    Math.hypot(mRobotBaseLength / 2.0, mRobotBaseWidth / 2.0);

    public static final double kFrontRightStartingEncoderPosition = 0;
    public static final double kFrontLeftStartingEncoderPosition = 0;
    public static final double kRearLeftStartingEncoderPosition = 0;
    public static final double kRearRightStartingEncoderPosition = 0;

    public static final boolean tuningMode = true;

    public static final boolean disableExtraTelemetry = true;

    public static final class ShooterConstants {
        public static final double HANDOFF = 0;
        public static final double IDLE = 0;
        public static final double FEEDFORWARD = 2;


        public static final double kShooterZeroAngle = 24;
        public static final double kShotTime = 1.2;
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> SHOT_TRAVEL_TIME_TREE_MAP = new InterpolatingTreeMap<>();
        public static final double kDeadband = 0;
        public static final double SPIN = 0.5;
        public static final double kShooterHeight = .14 ;
        public static final double kEncoderDistance = 0.113;
        static {
            SHOT_TRAVEL_TIME_TREE_MAP.put(new InterpolatingDouble(1.0), new InterpolatingDouble(0.0003));
            SHOT_TRAVEL_TIME_TREE_MAP.put(new InterpolatingDouble(6.0), new InterpolatingDouble(0.0045));
        }
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> SPEAKER_VELOCITY_TREE_MAP = new InterpolatingTreeMap<>();
        static {
            SPEAKER_VELOCITY_TREE_MAP.put(new InterpolatingDouble(1.0), new InterpolatingDouble(1.0));
            SPEAKER_VELOCITY_TREE_MAP.put(new InterpolatingDouble(6.0), new InterpolatingDouble(1.0));
        }
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> LOB_VELOCITY_TREE_MAP = new InterpolatingTreeMap<>();
        static {
            LOB_VELOCITY_TREE_MAP.put(new InterpolatingDouble(8.25), new InterpolatingDouble(0.6));
            LOB_VELOCITY_TREE_MAP.put(new InterpolatingDouble(16.0), new InterpolatingDouble(.6));
        }

    }

    public static final class IntakeConstants {
        public static final double RAMP = 0;
        public static final double HANDOFF = 0;
        public static final double IDLE = 0;
    }

    public static final class IndexerConstants {
        public static final double RAMP = 0;
        public static final double HANDOFF = 0;
        public static final double IDLE = 0;
    }

    public static final class ElevatorConstants {
        public static final double SPEAKER = 0;
        public static final double AMP = 0;
        public static final double TRAP = 0;
        public static final double TRANSFER = 0;
        public static final double SHOOTING = 0;
        public static final double MAX_UP = -80;
        public static final double MAX_DOWN = 0;
    }

    public static final class WristConstants {
        public static final double AMP = 0;
        public static final double TRAP = 0;
        public static final double TRANSFER = 0;
        public static final double SHOOTING = 0;
        public static final double MAX_UP = 0;
        public static final double MAX_DOWN = 0;
    }

    public static final class PivotConstants {
        public static final double SPEAKER = 0;
        public static final double AMP = 25;
        public static final double TRAP = 0;
        public static final double TRANSFER = 0;
        public static final double SHOOTING = 0;
        public static final double MAX_UP = 55;
        public static final double MAX_DOWN = 0;
        public static final double INTAKING = 10;

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> SpeakerAngleMap = new InterpolatingTreeMap<>();
        static {
            SpeakerAngleMap.put(new InterpolatingDouble(1.36), new InterpolatingDouble(23.5));//
            SpeakerAngleMap.put(new InterpolatingDouble(1.77), new InterpolatingDouble(18.5));//
            SpeakerAngleMap.put(new InterpolatingDouble(2.31), new InterpolatingDouble(11.5));//
            SpeakerAngleMap.put(new InterpolatingDouble(2.57), new InterpolatingDouble(10.5));//
            SpeakerAngleMap.put(new InterpolatingDouble(3.17), new InterpolatingDouble(6.0));//
            SpeakerAngleMap.put(new InterpolatingDouble(3.46), new InterpolatingDouble(4.5));//
            SpeakerAngleMap.put(new InterpolatingDouble(3.75), new InterpolatingDouble(4.0));//
            SpeakerAngleMap.put(new InterpolatingDouble(4.0), new InterpolatingDouble(3.0));//
            SpeakerAngleMap.put(new InterpolatingDouble(4.3), new InterpolatingDouble(2.0));//
            SpeakerAngleMap.put(new InterpolatingDouble(4.79), new InterpolatingDouble(1.0));//
            SpeakerAngleMap.put(new InterpolatingDouble(5.25), new InterpolatingDouble(.5));//
           SpeakerAngleMap.put(new InterpolatingDouble(6.16), new InterpolatingDouble(0.0));//

       }
        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> LobAngleMap = new InterpolatingTreeMap<>();
        static {

            LobAngleMap.put(new InterpolatingDouble(8.0), new InterpolatingDouble(20.96));
            LobAngleMap.put(new InterpolatingDouble(12.5), new InterpolatingDouble(20.96));
        }
    }

    public static final class ArmConstants {
        public static final double SPEAKER = 0;
        public static final double AMP = 55;
        public static final double TRAP = 0;
        public static final double TRANSFER = 0;
        public static final double SHOOTING = 0;
        public static final double MAX_UP = 0;
        public static final double MAX_DOWN = 0;
        public static final double PARTIAL = AMP/2;
    }

    public static final class VisionConstants {
        public static final int APRIL_PIPLINE = 0;
        public static final double LIMELIGHT_LENS_HEIGHT_INCHES = 15.75;

        public static final Pose2d ROBOT_TO_CAMERA = new Pose2d(new Translation2d(-.162-.23,0),
                Rotation2d.fromDegrees(0));

        public static final Rotation2d cameraYawOffset = Rotation2d.fromDegrees(0);
        public static final Rotation2d HORIZONTAL_PLANE_TO_LENSE = Rotation2d.fromDegrees(23);

        public static final double IMAGE_CAPTURE_LATENCY = 11.0;
       

        public static final UndistortConstants UNDISTORT_CONSTANTS =  new UndistortConstants(
                new double[]{0.15545342, -0.46477633, 0.00277053, 0.0030637, 0.39464241},
                new double[][]{{0.79862571, 0., 0.46119489},
                        {0., 1.06276288, 0.48098825},
                        {0., 0., 1.}}
                );

        public static final UndistortMap UNDISTORTMAP = new InterpolatingUndisortMap((int)1280.0, (int)960.0, new UndistortMap_Limelight_B_640x480());
    }

    public static final class LightConstants {
        public final static double NORMAL_LIGHT = -0.99;// rainbow light

        public final static double CUBE_LIGHT = 0.57;// purple light

        public final static double CONE_LIGHT = 0.65;// yellow light
    }
    
public static final class FieldConstants {

    public final static double kSpeakerHeight = 2.07;
    
         public static double[] reflect(double xs[]) {
            for(int i =0; i<xs.length; i++){
                xs[i] =16.5-xs[i];
            }
            return xs;
         }
        static double[] boxX = {2.88,5.83,5.83};
        static double[] boxY = {4.1,5.73,2.47};

        public static List<Obstacle> obstacles = List.of(
                new Obstacle(boxX, boxY),
                new Obstacle(reflect(boxX),boxY)
                );

        public static Pose2d getAmpPose() {
            if(DriverStation.getAlliance().get().equals(Alliance.Red)){
            return new Pose2d(16.5,8.19,new Rotation2d());
            }
            return new Pose2d(1.79,8.19,new Rotation2d());
        }
            }

  
    public static Pose2d getSpeakerAimingPose(){
        if(DriverStation.getAlliance().get().equals(Alliance.Red)){
            return new Pose2d(16.7,5.56,new Rotation2d());
        }
        return new Pose2d(16.7-16.28,5.56,new Rotation2d());
    }
    public static Pose2d getSpeakerPivotPose(){
        if(DriverStation.getAlliance().get().equals(Alliance.Red)){
            return new Pose2d(16.5,5.56,new Rotation2d());
        }
        return new Pose2d(0,5.56,new Rotation2d());
    }
    public static Pose2d getLobPose(){
        if(DriverStation.getAlliance().get().equals(Alliance.Red)){
            return new Pose2d(15.77,7.43,new Rotation2d());
        }
        return new Pose2d(16.5-15.77,7.43,new Rotation2d());
    }
 /**
     * Check if this system has a certain mac address in any network device.
     * @param mac_address Mac address to check.
     * @return true if some device with this mac address exists on this system.
     */
    public static boolean hasMacAddress(final String mac_address) {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis == null) {
                    continue;
                }
                StringBuilder device_mac_sb = new StringBuilder();
                System.out.println("hasMacAddress: NIS: " + nis.getDisplayName());
                byte[] mac = nis.getHardwareAddress();
                if (mac != null) {
                    for (int i = 0; i < mac.length; i++) {
                        device_mac_sb.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
                    }
                    String device_mac = device_mac_sb.toString();
                    System.out.println("hasMacAddress: NIS " + nis.getDisplayName() + " device_mac: " + device_mac);
                    if (mac_address.equals(device_mac)) {
                        System.out.println("hasMacAddress: ** Mac address match! " + device_mac);
                        return true;
                    }
                } else {
                    System.out.println("hasMacAddress: Address doesn't exist or is not accessible");
                }
            }

        } catch (SocketException e) {
            e.printStackTrace();
        }
        return false;
    }
}
