// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.uni.frc.Constants;
import com.uni.frc.Options;
import com.uni.frc.Ports;
import com.uni.frc.subsystems.Subsystem;
import com.uni.frc.subsystems.encoders.CANEncoder;
import com.uni.frc.subsystems.encoders.Encoder;
import com.uni.frc.subsystems.encoders.MagEncoder;
import com.uni.lib.Conversions;
import com.uni.lib.TalonConfigs;
import com.uni.lib.geometry.Pose2d;
import com.uni.lib.geometry.Rotation2d;
import com.uni.lib.geometry.Translation2d;
import com.uni.lib.swerve.SwerveModuleState;
import com.uni.lib.util.Util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Add your docs here. */
public class SwerveDriveModule extends Subsystem {

    TalonFX rotationMotor, driveMotor;
    Encoder rotationMagEncoder;
    String name;
    public int moduleID;
    double encoderOffset;

    private double previousEncDistance = 0;
	private Translation2d position = new Translation2d();
    private Translation2d mstartingPosition;

	private Pose2d estimatedRobotPose = new Pose2d();
	boolean standardCarpetDirection = true;
    
    Translation2d modulePosition;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV,
            Constants.driveKA);

    boolean rotationEncoderFlipped;

     ModuleIO mPeriodicIO = new ModuleIOAutoLogged();
    

    /**
     * 
     * @param rotationMotorPort  -The Drive Motor Port
     * @param driveMotorPort     -The Drive Motor Port
     * @param moduleID           -The ID of the module
     * @param encoderStartingPos -The starting encoder position(used for zeroing
     *                           purposes)
     * @param modulePoseInches         -The position of the module relative to the robot's
     *                           center
     * @param flipEncoder        -Is the encoder going in the right direction?
     *                           (clockwise = increasing, counter-clockwise =
     *                           decreasing)
     */
    public SwerveDriveModule(int rotationMotorPort, int driveMotorPort, int moduleID, double encoderStartingPos,
            Translation2d modulePoseInches, boolean flipEncoder ,Translation2d moduleposemeters){
        this.rotationMotor = new TalonFX(rotationMotorPort, Constants.isCompbot? "Minivore": "");
        this.driveMotor = new TalonFX(driveMotorPort, Constants.isCompbot? "Minivore": "");
        this.moduleID = moduleID;
        this.name = "Module " + moduleID;
        this.encoderOffset = encoderStartingPos;
        this.modulePosition = modulePoseInches;
        this.mstartingPosition =moduleposemeters;


        this.rotationEncoderFlipped = flipEncoder;

        if (Options.encoderType == "Mag Encoder") {
            rotationMagEncoder = new MagEncoder(Ports.SWERVE_ENCODERS[moduleID]);
        } 
        else if(Options.encoderType == "CANCoder") {
         rotationMagEncoder = new CANEncoder(Ports.SWERVE_ENCODERS[moduleID]);
        }
        configMotors();
    }
    TalonFXConfiguration driveConfigs = new TalonFXConfiguration();
    TalonFXConfiguration rotationConfigs = new TalonFXConfiguration();

    public void configMotors(){
        driveConfigs = TalonConfigs.swerveDriveConfig();
        rotationConfigs = TalonConfigs.swerveRotationConfig();

        driveMotor.getConfigurator().apply(driveConfigs);
        rotationMotor.getConfigurator().apply(rotationConfigs);

        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        rotationMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotor.setPosition(0);

    }

    public enum ControlMode{
        PercentOuput,
        MotionMagic,
        Velocity
      }

    public void invertDriveMotor(boolean invert) {
        driveMotor.setInverted(invert);
    }

    public void invertRotationMotor(Boolean invertType) {
        rotationMotor.setInverted(invertType);
    }

    public void invertRotationMotor(boolean invert) {
        rotationMotor.setInverted(invert);
    }

    public void setDriveMotorNeutralMode(NeutralModeValue mode) {
        driveMotor.setNeutralMode(mode);
    }

    public void setModuleAngle(double desiredAngle) {
        desiredAngle = Util.placeInAppropriate0To360Scope(getModuleAngle(), desiredAngle);// optimization
        double angleRotations = degreesToRotations(desiredAngle);
        mPeriodicIO.rotationDemand = angleRotations;
    }

    public double getModuleAngle() {
        return rotationsToDegrees(mPeriodicIO.rotationPosition);
    }

    public double getModuleAbsolutePosition() {
        return rotationMagEncoder.getOutput() * ((this.rotationEncoderFlipped) ? -1 : 1) * 360.0;
    }

    public boolean isMagEncoderConnected() {
        return rotationMagEncoder.isConnected();
    }



    public void resetEncoders() {
        driveMotor.setPosition(0);

    }
    public synchronized void resetPose(Pose2d robotPose){
		Translation2d modulePosition = robotPose.transformBy(Pose2d.fromTranslation(mstartingPosition)).getTranslation();
		position = modulePosition;
	}

    public Pose2d getEstimatedRobotPose(){
		return estimatedRobotPose;
	}

    public Rotation2d getFieldCentricAngle(Rotation2d robotHeading){
		Rotation2d normalizedAngle =Rotation2d.fromDegrees( getModuleAngle());
		return normalizedAngle.rotateBy(robotHeading);
	}
	
    public double encUnitsToInches(double encUnits){
		return encUnits/Constants.kSwerveEncUnitsPerInch;
	}
    public void setVelocityPercent(double percent){
        mPeriodicIO.driveDemand = percent*Constants.SwerveMaxspeedMPS;
        mPeriodicIO.driveControlMode = ControlMode.Velocity;
    }
    
    public void setDriveOpenLoop(double percentOuput) {
        mPeriodicIO.driveControlMode = ControlMode.PercentOuput;
        mPeriodicIO.driveDemand = percentOuput;
    }

    public void setDriveVelocity(double velocity) {
        mPeriodicIO.driveControlMode = ControlMode.Velocity;
        mPeriodicIO.driveDemand = (velocity/Constants.kWheelCircumference)*Options.driveRatio;
    }

    public SwerveModuleState getSwerveModuleState(){
        return new SwerveModuleState(Conversions.falconToMPS(
            mPeriodicIO.velocity,
            Constants.kWheelCircumference,
            Options.driveRatio),
            mPeriodicIO.distanceTraveled ,
            Rotation2d.fromDegrees(rotationsToDegrees(mPeriodicIO.rotationPosition)
            ));
    }
    
    public synchronized void updatePose(Rotation2d robotHeading){
		double currentEncDistance = Conversions.falconToMeters(driveMotor.getPosition().getValueAsDouble(), Constants.kWheelCircumference, Options.driveRatio);
		double deltaEncDistance = (currentEncDistance - previousEncDistance) * Constants.kWheelScrubFactors[moduleID];
		Rotation2d currentWheelAngle = getFieldCentricAngle(robotHeading);
		Translation2d deltaPosition = new Translation2d(-currentWheelAngle.cos()*deltaEncDistance, 
				currentWheelAngle.sin()*deltaEncDistance);


		double xScrubFactor = Constants.kXScrubFactorN;
		double yScrubFactor = Constants.kYScrubFactorN;
        if(Util.epsilonEquals(Math.signum(deltaPosition.x()), 1.0)){
            xScrubFactor = Constants.kXScrubFactorP;

        }
        if(Util.epsilonEquals(Math.signum(deltaPosition.y()), 1.0)){
            yScrubFactor = Constants.kYScrubFactorP;
        }
	
		deltaPosition = new Translation2d(deltaPosition.x() * xScrubFactor,
			deltaPosition.y() * yScrubFactor);
        mPeriodicIO.distanceTraveled += deltaPosition.norm();
		Translation2d updatedPosition = position.translateBy(deltaPosition);
		Pose2d staticWheelPose = new Pose2d(updatedPosition, robotHeading);
		Pose2d robotPose = staticWheelPose.transformBy(Pose2d.fromTranslation(mstartingPosition).inverse());
		position = updatedPosition;
		estimatedRobotPose =  robotPose;
		previousEncDistance = currentEncDistance;
	}

    public double degreesToRotations(double degrees){
        return (degrees/360)*Options.rotationRatio;
    }

    public double rotationsToDegrees(double rotations){
        return (rotations*360)/Options.rotationRatio;
    }

    public void resetModulePositionToAbsolute() {
                driveMotor.setPosition(0);
                double offset = getModuleAbsolutePosition() - encoderOffset;
                rotationMotor.setPosition(degreesToRotations(offset));
    }

    public enum ModuleStatus {
        OK, ABSOLUTE_ENCODER_ERROR, DRIVE_MOTOR_ERROR, ROTATION_MOTOR_ERROR;
    }

   

    public ModuleStatus getModuleStatus() {
        if (!isMagEncoderConnected())
            return ModuleStatus.ABSOLUTE_ENCODER_ERROR;
        else if (driveMotor.isAlive())
            return ModuleStatus.DRIVE_MOTOR_ERROR;
        else if (rotationMotor.isAlive())
            return ModuleStatus.ROTATION_MOTOR_ERROR;
        return ModuleStatus.OK;
    }

    @Override
    public void writePeriodicOutputs() {
        mPeriodicIO.rotationPosition = rotationMotor.getPosition().getValue();
        mPeriodicIO.drivePosition = driveMotor.getPosition().getValue();
        mPeriodicIO.velocity = driveMotor.getVelocity().getValue();
    }

      @Override
  public void readPeriodicInputs() {
    switch (mPeriodicIO.driveControlMode) {
      case PercentOuput:
          runPercentOutput(mPeriodicIO.driveDemand, driveMotor);
        break;
      case MotionMagic:
          runMotionMagic(mPeriodicIO.driveDemand, driveMotor);
        break;
      case Velocity:
          runVelocity(mPeriodicIO.driveDemand, driveMotor);
        break;
    }
    switch (mPeriodicIO.rotationControlMode) {
      case PercentOuput:
          runPercentOutput(mPeriodicIO.rotationDemand, rotationMotor);
        break;
      case MotionMagic:
          runMotionMagic(mPeriodicIO.rotationDemand, rotationMotor);
        default:
          runMotionMagic(mPeriodicIO.rotationDemand, rotationMotor);
        break;
    }

  }

  public void runPercentOutput(double percent, TalonFX motor){
    motor.setControl(new DutyCycleOut(percent, true, false, false, false));
  }
    public void runMotionMagic(double position, TalonFX motor){
    motor.setControl(new MotionMagicVoltage(position).withEnableFOC(false ));
  }

  public void runVelocity(double velocity, TalonFX motor){
    motor.setControl(new MotionMagicVelocityVoltage(velocity).withSlot(0).withEnableFOC(true));
  }
    @Override
    public void outputTelemetry() {
        Logger.recordOutput("Swerve/"+ this.name + "/Angle Demand", rotationsToDegrees(mPeriodicIO.rotationDemand));
        Logger.recordOutput("Swerve/"+ this.name + "/Angle", rotationsToDegrees(mPeriodicIO.rotationPosition));
        Logger.recordOutput("Swerve/"+ this.name + "/Absolute Position", getModuleAbsolutePosition());
        Logger.recordOutput("Swerve/"+ this.name + "/Drive Motor Demand", mPeriodicIO.driveDemand);
        Logger.recordOutput("Swerve/"+ this.name + "/Status", getModuleStatus().toString());
        Logger.recordOutput("Swerve/"+ this.name + "/Drive velocity", driveMotor.getVelocity().getValueAsDouble());
        Logger.recordOutput("Swerve/"+ this.name + "/Demanded Drive", mPeriodicIO.driveDemand);


    }

    @Override
    public void stop() {// stops everything
        setModuleAngle(getModuleAngle());
        setDriveOpenLoop(0.0);
    }
    @AutoLog
    public static class ModuleIO {
        double rotationPosition = 0;
        double drivePosition = 0;
        double velocity = 0;
        double distanceTraveled = 0;

        ControlMode rotationControlMode = ControlMode.MotionMagic;
        ControlMode driveControlMode = ControlMode.PercentOuput;
        double rotationDemand = 0.0;
        double driveDemand = 0.0;
    }

}
