// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc;

/** Add your docs here. */
public class Options {
    /// * REPLACE!!!!! *///
    public static final boolean isCompbot = Constants.isCompbot;
    public static final String encoderType = isCompbot ?  "CANCoder" : "Mag Encoder";
    public static final String motorType = "Falcon 500";
    public static final double rotationRatio = isCompbot ? 11.3142 : 15.4285714285714;
    public static final double driveRatio = isCompbot ? 4.41 : 6.25;
    /// * REPLACE!!!!! *////

}
