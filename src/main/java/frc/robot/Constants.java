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
    public static final double RAMSETE_B = 2.0;
    public static final double RAMSETE_ZETA =  0.7;
}
