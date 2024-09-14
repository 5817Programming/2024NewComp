// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc;

/** Add your docs here. */// new
public class Ports {
    public static Boolean isCompbot = Constants.isCompbot;
    public static final int XBOX_1 = 0;
    public static final int XBOX_2 = 1;

    public static final int FRONT_RIGHT_ROTATION = isCompbot?5:13;
    public static final int FRONT_RIGHT_DRIVE = isCompbot?1:12;

    public static final int FRONT_LEFT_ROTATION = isCompbot?4:6; 
    public static final int FRONT_LEFT_DRIVE =isCompbot?0: 0; 

    public static final int REAR_LEFT_ROTATION = isCompbot?7:8; 
    public static final int REAR_LEFT_DRIVE = isCompbot?3:7; 

    public static final int REAR_RIGHT_ROTATION = isCompbot?6:11;
    public static final int REAR_RIGHT_DRIVE = isCompbot?2:10;

    public static final int PIGEON = isCompbot?23:23;

    public static final int FRONT_RIGHT_ENCODER = isCompbot?9:9;
    public static final int FRONT_LEFT_ENCODER = isCompbot?8:7;
    public static final int REAR_LEFT_ENCODER = isCompbot?11:6;
    public static final int REAR_RIGHT_ENCODER = isCompbot?10:8;
    public static final int[] SWERVE_ENCODERS = new int[] {FRONT_RIGHT_ENCODER, FRONT_LEFT_ENCODER,
            REAR_LEFT_ENCODER, REAR_RIGHT_ENCODER };

    public static final int indexerBeam = isCompbot?0:0;
    
    public static final int PivotEncoder = 12;
   
        public static final int Intake = 22;
        public static final int shooter1 = 18;
        public static final int shooter2 = 26;
        public static final int Pivot1 = 13;
        public static final int Pivot2 = 14;

        public static final int Indexer = 19; 
        public static final int IndexerBeamBreakPort = 0;
        public static final int elevatorMotor1 = 20;
        public static final int elevatorMotor2 = 21;
        public static final int Arm = 24;
        public static final int Lights = 9;




}
