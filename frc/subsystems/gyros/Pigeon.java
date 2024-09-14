// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.uni.frc.subsystems.gyros;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.uni.frc.Constants;
import com.uni.frc.Ports;
import com.uni.lib.geometry.Twist2d;


/** 
 * This class is used as an extention of the original pidgeon class
 * we improved on their methods by offsetting the error on get pitch and by inverting getangle in order to better suit our robots needs
 */
public class Pigeon extends Gyro {
    private static Pigeon instance = null;

    public static Pigeon getInstance() {
        if (instance == null)
            instance = new Pigeon();
        return instance;
    }

    Pigeon2 pigeon;

    public Pigeon() {
        try {
            pigeon = new Pigeon2(Ports.PIGEON, Constants.isCompbot?"Minivore":"");
            // secondPigeon = new PigeonIMU(Ports.SECONDARY_PIGEON);
        } catch (Exception e) {
            System.out.println(e);
        }
    }

    public double getPitch() {
        return pigeon.getRoll().getValue() + 1.7;
    }

    public void setAngle(double angle) {
        pigeon.setYaw(angle);
        // pigeon.setFusedHeading(-angle * 64.0, Constants.kCANTimeoutMs);

    }

    public Twist2d getVelocity(){
        var x = pigeon.getAccelerationX().getValue()*9.80665;
        var y = pigeon.getAccelerationX().getValue()*9.80665;
        var dt = pigeon.getAngularVelocityZWorld().getValue();
        return new Twist2d(x, y, dt);
    }

    @Override
    public double getAngle() {
        return -pigeon.getYaw().getValue();
    }
}
