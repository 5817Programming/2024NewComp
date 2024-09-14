// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.lib.swerve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.ejml.simple.SimpleMatrix;
import org.opencv.video.TrackerDaSiamRPN;

import com.uni.frc.Constants;
import com.uni.frc.subsystems.Swerve.SwerveDriveModule;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;

/** Add your docs here. */
public class SwerveKinematics {

    private final SimpleMatrix mInverseKinematics;
    private final SimpleMatrix mForwardKinematics;
    private final int m_numModules = Constants.kModulePositions.size();


    private Translation2d m_prevCoR = new Translation2d();
    private SwerveModuleState[] mModuleStates;
    private Translation2d[] mModules;

    private List<Translation2d> moduleRotationVectors;
    private List<Rotation2d> moduleRotations = new ArrayList<>(m_numModules);



    public SwerveKinematics() {
        updateModuleRotationVectors();
        Translation2d[] wheelMeters = Constants.mModulePositions;
        mModuleStates = new SwerveModuleState[m_numModules];

        mModules = Arrays.copyOf(wheelMeters, m_numModules);

        Arrays.fill(mModuleStates, new SwerveModuleState());
        mInverseKinematics = new SimpleMatrix(m_numModules * 2, 3);

        for (int i = 0; i < m_numModules; i++) {
            mInverseKinematics.setRow(i * 2 + 0, 0, /* Start Data */ 1, 0, -mModules[i].y());
			mInverseKinematics.setRow(i * 2 + 1, 0, /* Start Data */ 0, 1, +mModules[i].x());
		}
		mForwardKinematics = mInverseKinematics.pseudoInverse();
    }
 

    
    public void updateModuleRotationVectors() {
        int numberOfModules = m_numModules;
        double rotateVectorDirection = 360.0 / numberOfModules;
        List<Translation2d> vectorList = new ArrayList<>(numberOfModules);
        for(int i = 0; i < numberOfModules; i++) {
            vectorList.add(Constants.kModulePositions.get(i).rotateBy(Rotation2d.fromDegrees(rotateVectorDirection)));
            moduleRotations.add(Constants.kModulePositions.get(i).getAngle());
        }
        moduleRotationVectors = vectorList;
    }
    /**
     * 
     * @param translationVector -A vector that that is usually represented by the translational stick on the controller
     * @param rotationalMagnitude -The amount of rotation of the swerve drive represented by the rotational stick on the controller
     * @param robotPosition -The Position of the robot, with respect to the Rotation
     * @param robotCentric -Should the determine it's heading based off of the robot. False = Field Centric(i.e, when you push forward, 
     *                      the robot will always move in the same direction, no matter it's heading)
     */
    public List<Translation2d> updateDriveVectors(Translation2d translationVector, double rotationalMagnitude, Pose2d robotPosition, boolean robotCentric) {
        List<Translation2d> driveVectors = new ArrayList<>(m_numModules);
        if(!robotCentric)
            translationVector = translationVector.rotateBy(robotPosition.getRotation().inverse()); //Rotates by the translation vector by inverse rotation of the robot 
        for(int i = 0; i < m_numModules; i++) {
            driveVectors.add(translationVector.translateBy(moduleRotationVectors.get(i).scale(rotationalMagnitude))); //Rotates the translation vector of each of the modules, by the rotation value
        }

        double maxMagnitude = 1.0;
        for (Translation2d m : driveVectors) {
            double moduleVectorMagnitude = m.norm();
            if(moduleVectorMagnitude > maxMagnitude)
                maxMagnitude = moduleVectorMagnitude;
        }
        for (int i =0; i< m_numModules; i++) {
            Translation2d driveVector = driveVectors.get(i);
            driveVectors.set(i, driveVector.scale(1.0/maxMagnitude));
        }
        return driveVectors;
        
    }

    public static void desaturateWheelSpeeds(
			SwerveModuleState[] moduleStates, double attainableMaxSpeedMetersPerSecond) {
		double realMaxSpeed = Collections.max(Arrays.asList(moduleStates)).speedMetersPerSecond;
		if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
			for (SwerveModuleState SwerveModuleState : moduleStates) {
				SwerveModuleState.speedMetersPerSecond =
						SwerveModuleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
			}
		}
	}

    public ChassisSpeeds toChassisSpeedWheelConstraints(List<SwerveDriveModule> modules) {
        if (modules.size() != m_numModules) {
            throw new IllegalArgumentException(
                    "Number of modules is not consistent with number of wheel locations provided in "
                            + "constructor");
        }
        var constraintsMatrix = new SimpleMatrix(m_numModules * 2, 3);
        for (int i = 0; i < m_numModules; i++) {
            SwerveModuleState module = modules.get(i).getSwerveModuleState();

            Rotation2d beta =
                    module.angle.rotateBy(moduleRotations.get(i).inverse()).rotateBy(Rotation2d.fromRadians(Math.PI / 2.0));

            //System.out.println(module);
            constraintsMatrix.setRow(i*2, 0,
                    module.angle.cos(),
                    module.angle.sin(),
                    -Constants.kModulePositions.get(i).norm()*beta.cos());
            constraintsMatrix.setRow(i*2 + 1, 0,
                    -module.angle.sin(),
                    module.angle.cos(),
                    Constants.kModulePositions.get(i).norm()*beta.sin());
        }

        var psuedoInv = constraintsMatrix.pseudoInverse();

        var enforcedConstraints = new SimpleMatrix(m_numModules*2, 1);
        for (int i = 0; i < m_numModules; i++) {
            enforcedConstraints.setRow(i*2, 0, modules.get(i).getSwerveModuleState().speedMetersPerSecond);
            enforcedConstraints.setRow(i*2 + 1, 0, 0);
        }
        //System.out.println(enforcedConstraints);

        var chassisSpeedsVector = psuedoInv.mult(enforcedConstraints);
        return new ChassisSpeeds(
                chassisSpeedsVector.get(0, 0),
                chassisSpeedsVector.get(1, 0),
                chassisSpeedsVector.get(2, 0));
    }
    	@SuppressWarnings("PMD.MethodReturnsInternalArray")
	public SwerveModuleState[] toModuleStates(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotationMeters) {
		if (chassisSpeeds.vxMetersPerSecond == 0.0
				&& chassisSpeeds.vyMetersPerSecond == 0.0
				&& chassisSpeeds.omegaRadiansPerSecond == 0.0) {
			for (int i = 0; i < m_numModules; i++) {
				mModuleStates[i].speedMetersPerSecond = 0.0;
			}

			return mModuleStates;
		}

		if (!centerOfRotationMeters.equals(m_prevCoR)) {
			for (int i = 0; i < m_numModules; i++) {
				mInverseKinematics.setRow(
						i * 2 + 0, 0, /* Start Data */ 1, 0, -mModules[i].y() + centerOfRotationMeters.y());
				mInverseKinematics.setRow(
						i * 2 + 1, 0, /* Start Data */ 0, 1, +mModules[i].x() - centerOfRotationMeters.x());
			}
			m_prevCoR = centerOfRotationMeters;
		}

		var chassisSpeedsVector = new SimpleMatrix(3, 1);
		chassisSpeedsVector.setColumn(
				0,
				0,
				chassisSpeeds.vxMetersPerSecond,
				chassisSpeeds.vyMetersPerSecond,
				chassisSpeeds.omegaRadiansPerSecond);

		var moduleStatesMatrix = mInverseKinematics.mult(chassisSpeedsVector);

		for (int i = 0; i < m_numModules; i++) {
			double x = moduleStatesMatrix.get(i * 2, 0);
			double y = moduleStatesMatrix.get(i * 2 + 1, 0);

			double speed = Math.hypot(x, y);
			Rotation2d angle = new Rotation2d(x, y, true);

			mModuleStates[i] = new SwerveModuleState(speed, angle);
		}

		return mModuleStates;
	}

	/**
	 * Performs inverse kinematics. See {@link #toModuleStates(ChassisSpeeds, Translation2d)}
	 * toModuleStates for more information.
	 *
	 * @param chassisSpeeds The desired chassis speed.
	 * @return An array containing the module states.
	 */
	public SwerveModuleState[] toModuleStates(ChassisSpeeds chassisSpeeds) {
		return toModuleStates(chassisSpeeds, new Translation2d());
	}
}
