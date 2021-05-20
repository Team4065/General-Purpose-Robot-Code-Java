/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * This is where the robot and the controller inputs are configured CAN IDs must
 * not repeat or else it will result in unexpected behaviors
 */
public class Constants {
    public static final boolean IS_GYRO_REVERSED = true;
    public static final boolean IS_SPY_ENABLED = true;


    //Ramsete
    public static final double ROBOT_TRACKWIDTH = 0.5969;
    public static final double RAMSETE_B = 2.0;
    public static final double RAMSETE_ZETA = 0.7;
    //public static final double KS_VOLTS = 1.1232140651648732;
    //public static final double KV_VOLT_SECONDS_PER_METER = 2.2958171993801137;
    //public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = -0.04163995848262118;
    public static final DifferentialDriveKinematics DIFFERENTIAL_DRIVE_KINEMATICS = new DifferentialDriveKinematics(ROBOT_TRACKWIDTH);
    //public static final double KP_DRIVE_VEL = 0;
}
