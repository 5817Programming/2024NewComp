package com.uni.lib.util;
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package com.wcp.lib.util;

// import org.littletonrobotics.junction.Logger;
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
// import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.wcp.frc.Constants;
// import com.wcp.frc.subsystems.Subsystem;
// import com.wcp.frc.subsystems.Requests.Request;

// /**
//  * This class uses motion magic
//  * moton magic is used to smoothly go to a specified postion 
//  * we use the motion magic libarary to set the arm motor to a specific rotation
//  */
// public class MotionMagicMotor extends Subsystem {
//     int port;
// 	double deadband = 0;


	
//     public MotionMagicMotor(NeutralMode neutralMode, boolean phaseSensor, boolean invert,double kP,double kF,double kI,double cruiseVelocity, double acceleration, int port, double deadband){
// 	/* Factory 
// 		default hardware to prevent unexpected behavior */
// 		motor.configFactoryDefault();
// 		// mRightElevator.configFactoryDefault();

// 		/* Set to Brake Mode */
// 		motor.setNeutralMode(neutralMode);
// 		// mRightElevator.setNeutralMode(NeutralMode.Brake);

// 		/* Will follow so no need to config further */
// 		// mRightElevator.follow(mLeftElevator);
// 		// mRightElevator.setInverted(TalonFXInvertType.FollowMaster);

// 		/* Configure Sensor Source for Pirmary PID */
// 		motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
// 				Constants.TIMEOUT_MILLISECONDS);

// 		/* set deadband to super small 0.001 (0.1 %).
// 			The default deadband is 0.04 (4 %) */
// 		motor.configNeutralDeadband(0.001, Constants.TIMEOUT_MILLISECONDS);

// 		/**
// 		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
// 		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
// 		 * sensor to have positive increment when driving Talon Forward (Green LED)
// 		 */
// 		motor.setSensorPhase(phaseSensor);
// 		motor.setInverted(invert);

// 		/*
// 		 * Talon FX does not need sensor phase set for its integrated sensor
// 		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
// 		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
// 		 * 
// 		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
// 		 */
//         // mLeftElevator.setSensorPhase(true);

// 		/* Set relevant frame periods to be at least as fast as periodic rate */
// 		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.TIMEOUT_MILLISECONDS);
// 		motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.TIMEOUT_MILLISECONDS);

// 		/* Set the peak and nominal outputs */
// 		motor.configNominalOutputForward(0, Constants.TIMEOUT_MILLISECONDS);
// 		motor.configNominalOutputReverse(0, Constants.TIMEOUT_MILLISECONDS);
// 		motor.configPeakOutputForward(1, Constants.TIMEOUT_MILLISECONDS);
// 		motor.configPeakOutputReverse(-1, Constants.TIMEOUT_MILLISECONDS);
		
// 		/* Set Motion Magic gains in slot0 - see documentation */
// 		motor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
// 		motor.config_kF(Constants.kSlotIdx, kF, Constants.TIMEOUT_MILLISECONDS);//0.0649
// 		motor.config_kP(Constants.kSlotIdx, kP, Constants.TIMEOUT_MILLISECONDS);//0.7161
// 		motor.config_kI(Constants.kSlotIdx, kI, Constants.TIMEOUT_MILLISECONDS);//0.001
// 		motor.config_kD(Constants.kSlotIdx, kP*10, Constants.TIMEOUT_MILLISECONDS);//P value * ten

// 		/*mLeftElevator.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
// 		mLeftElevator.config_kF(Constants.kSlotIdx, 0.05757217626, Constants.kTimeoutMs);//0.0649
// 		mLeftElevator.config_kP(Constants.kSlotIdx, 0, Constants.kTimeoutMs);//0.7161
// 		mLeftElevator.config_kI(Constants.kSlotIdx, 0, Constants.kTimeoutMs);//0.001
// 		mLeftElevator.config_kD(Constants.kSlotIdx, 0, Constants.kTimeoutMs);//P value * ten
// */
// 		/* Set acceleration and vcruise velocity - see documentation */
// 		motor.configMotionCruiseVelocity(cruiseVelocity, Constants.TIMEOUT_MILLISECONDS);
// 		motor.configMotionAcceleration(acceleration, Constants.TIMEOUT_MILLISECONDS);

// 		/* Zero the sensor once on robot boot up */
// 		motor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.TIMEOUT_MILLISECONDS);

//         this.port = port;
// 	}

	
// 	PeriodicIO mPeriodicIO = new PeriodicIO();

// 	/* Setpoints */
// 	double mTargetMin;
// 	double mTargetMax;
// 	double mtargetmid;
// 	double targetPos;
// 	public double velocity;

// 	/* Hardware */
// 	TalonFX motor = new TalonFX(this.port);

// 	/* Used to build string throughout loop */
// 	StringBuilder _sb = new StringBuilder();

// 	/** How much smoothing [0,8] to use during MotionMagic */
// 	int _smoothing = 0;

// 	/** save the last Point Of View / D-pad value */
// 	int _pov = -1;

	

// 	//this sets the position for motion magic to set the arm to 
// 	public void setMotionMagic(double targetPos) {
// 		mPeriodicIO.driveControlMode = ControlMode.MotionMagic;
// 		mPeriodicIO.driveDemand = targetPos;

// 	}

// 	public synchronized boolean isFinished(){
// 		return Math.abs(mPeriodicIO.driveDemand-mPeriodicIO.drivePosition)<deadband;
// 	}



// 	//sets the percent of power the motor will output
// 	public Request percentRequest(double percent){
// 		return new Request() {
// 			@Override
// 				public void act() {
// 					setPercentOutput(percent);
// 				}
// 			@Override
// 				public boolean isFinished(){
// 					return true;
// 				}
// 		};
// 	}

	
// 	//sets the percent of power the motor will output
// 	public void setPercentOutput(double speed) {
// 		mPeriodicIO.driveControlMode = ControlMode.PercentOutput;
// 		mPeriodicIO.driveDemand = speed;
// 	}

// 	public void configVelocity(double velocity) {
// 		motor.configMotionCruiseVelocity(velocity, Constants.TIMEOUT_MILLISECONDS);

// 	}

// 	//returns the position of the arm in ticks
// 	public double getEncoder(){
// 		return mPeriodicIO.drivePosition;
// 	  }

// 	  //updates the periodic IO's knowlage of the state of the robot to stay organized
//     public void writePeriodicOutputs() {
//         mPeriodicIO.drivePosition = motor.getSelectedSensorPosition();
//         mPeriodicIO.velocity = motor.getSelectedSensorVelocity();
//     }

// 	//Listens to what the periodic IO tells it to do
//     public void readPeriodicInputs() {
//         motor.set(mPeriodicIO.driveControlMode, mPeriodicIO.driveDemand);
//     }

// 	public static class PeriodicIO  {
//         double drivePosition = 0;
//         double velocity = 0;

//         ControlMode driveControlMode = ControlMode.MotionMagic;
//         double driveDemand = 0.0;
//     }


// 	@Override
// 	public void stop() {
// 		setPercentOutput(0);
// 	}



//     @Override
//     public void outputTelemetry() {
//     }

// }
 