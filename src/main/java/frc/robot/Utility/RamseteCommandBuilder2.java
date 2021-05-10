// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DifferentialDrivetrain2;

/** Add your docs here. */
public class RamseteCommandBuilder2 {
    private RamseteCommand command;
    private DifferentialDrivetrain2 m_drivetrain;
    private Trajectory m_trajectory;


    RamseteController disabledRamsete = new RamseteController() {
        @Override
        public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                double angularVelocityRefRadiansPerSecond) {
            return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
        }
    };

    /**
     * Makes the Ramsete command that is to be run.
     * @param drivetrain The drivetrain subsystem.
     * @param path The PathLoader with the desired Ramsete path.
     */
    public RamseteCommandBuilder2(DifferentialDrivetrain2 drivetrain, String pathName){
        command = new RamseteCommand(
            (new PathLoader(pathName)).getTrajectory(), 
            drivetrain::getPose,
            disabledRamsete,
            //new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
            Constants.DIFFERENTIAL_DRIVE_KINEMATICS,
            drivetrain::setTankDriveVelocity,
            drivetrain);
        m_drivetrain = drivetrain;
        m_trajectory = (new PathLoader(pathName)).getTrajectory();
    }

    public RamseteCommand getCommand(){
        m_drivetrain.resetOdometry(m_trajectory.getInitialPose());
        return command;
    }
}
