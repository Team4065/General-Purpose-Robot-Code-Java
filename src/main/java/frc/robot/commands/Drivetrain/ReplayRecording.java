// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utility.Motors.Motor.ControlMode;
import frc.robot.subsystems.DifferentialDrivetrain2;

public class ReplayRecording extends CommandBase {
  DifferentialDrivetrain2 m_drivetrain;
  Double[] m_leftRecording;
  Double[] m_rightRecording;
  int counter;
  
  /** Creates a new ReplayRecording. */
  public ReplayRecording(DifferentialDrivetrain2 drivetrain, Double[] leftRecording, Double[] rightRecording) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_leftRecording = leftRecording;
    m_rightRecording = rightRecording;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    m_drivetrain.setControlMode(ControlMode.Velocity);
    m_drivetrain.setLeftTarget(0);
    m_drivetrain.setRightTarget(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setLeftTarget(m_leftRecording[counter]);
    m_drivetrain.setRightTarget(m_rightRecording[counter]);

    ++counter;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setControlMode(ControlMode.PercentOutput);
    m_drivetrain.setLeftTarget(0);
    m_drivetrain.setRightTarget(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return counter >= m_leftRecording.length;
  }
}
