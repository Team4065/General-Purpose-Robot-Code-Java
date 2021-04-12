/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility;

import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Add your docs here.
 */
public class Gyro {
    private static AHRS gyro = new AHRS(SPI.Port.kMXP);
    private static double pastRate = 0;
    private static double acceleration = 0;

    /**
     * Returns the z-axis angle (heading) reported by the gyroscope with alterations from Constants settings.
     * @return
     */
    public static double getAngle(){
        return gyro.getAngle() * (Constants.IS_GYRO_REVERSED ? -1.0 : 1.0);
    }

    /**
     * Returns the z-axis angle (heading) reported by the gyroscope without alterations from Constants settings.
     * @return
     */
    public static double getRawAngle(){
        return gyro.getAngle();
    }

    /**
     * Returns the heading of the gyroscope based on a compass.
     * @return
     */
    public static double getCompassHeading(){
        return gyro.getCompassHeading() * (Constants.IS_GYRO_REVERSED ? -1.0 : 1.0);
    }

    /**
     * Calibrates the gyro
     * @return
     */
    public static void calibrate(){
        gyro.calibrate();
    }

    /**
     * Returns the heading of the gyroscope based on a compass and accelerometers
     * Reduces drift but less accurate than getAngle.
     * Faster than getCompassHeading.
     * @return
     */
    public static double getFusedHeading(){
        return gyro.getFusedHeading();
    }

    public static Rotation2d getRotation2d(){
        return gyro.getRotation2d();
    }

    public static void reset(){
        gyro.reset();
    }

    public static double getRate(){
        return Math.toDegrees(gyro.getRate()) * (Constants.IS_GYRO_REVERSED ? -1.0 : 1.0);
    }

    public static void updateAcceleration(){
        acceleration = (getRate() - pastRate) / (20. / 1000.);
        pastRate = getRate();
    }

    public static double getAcceleration(){
        return Gyro.acceleration;
    }
}
