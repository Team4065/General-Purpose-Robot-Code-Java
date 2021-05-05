/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility;

import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Add your docs here.
 */
public class Gyro {
    private static AHRS gyro = new AHRS(SPI.Port.kMXP);
    private static RomiGyro rGyro = new RomiGyro();
    private static Derivative acceleration = new Derivative(()->Gyro.getRate());

    /**
     * Returns the z-axis angle (heading) reported by the gyroscope with alterations from Constants settings.
     * @return
     */
    public static double getAngle(){
        if(RobotBase.isReal())
            return gyro.getAngle() * (Constants.IS_GYRO_REVERSED ? -1.0 : 1.0);
        else
            return rGyro.getAngleZ() * (Constants.IS_GYRO_REVERSED ? -1.0 : 1.0);
    }

    /**
     * Returns the z-axis angle (heading) reported by the gyroscope without alterations from Constants settings.
     * @return
     */
    public static double getRawAngle(){
        if(RobotBase.isReal())
            return gyro.getAngle();
        else
            return rGyro.getAngleZ();

    }

    /**
     * Returns the heading of the gyroscope based on a compass.
     * @return
     */
    public static double getCompassHeading(){
        if(RobotBase.isReal())
            return gyro.getCompassHeading() * (Constants.IS_GYRO_REVERSED ? -1.0 : 1.0);
        else
            return Double.NaN;
    }

    /**
     * Calibrates the gyro
     * @return
     */
    public static void calibrate(){
        if(RobotBase.isReal())
            gyro.calibrate();
    }

    /**
     * Returns the heading of the gyroscope based on a compass and accelerometers
     * Reduces drift but less accurate than getAngle.
     * Faster than getCompassHeading.
     * @return
     */
    public static double getFusedHeading(){
        if(RobotBase.isReal())
            return gyro.getFusedHeading();
        else
            return rGyro.getAngleZ();
    }

    public static Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(Gyro.getAngle());
    }

    public static void reset(){
        if(RobotBase.isReal())
            gyro.reset();
        else
            rGyro.reset();
    }

    public static double getRate(){
        if(RobotBase.isReal())
            return Math.toDegrees(gyro.getRate()) * (Constants.IS_GYRO_REVERSED ? -1.0 : 1.0);
        else
            return rGyro.getRateZ() * (Constants.IS_GYRO_REVERSED ? -1.0 : 1.0);
    }

    public static double getAcceleration(){
        return Gyro.acceleration.get();
    }
}
