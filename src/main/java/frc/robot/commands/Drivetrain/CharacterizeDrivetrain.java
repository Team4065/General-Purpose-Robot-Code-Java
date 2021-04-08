// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.DifferentialDrivetrain.ControlMode;

/**
 * To use run the command.
 * Copy the output into desmos and use linear regression to find the variables of the equation given at 
 * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html
 */


public class CharacterizeDrivetrain extends CommandBase {
  DifferentialDrivetrain m_drivetrain;
  double m_targetVoltage = 0;
  double m_pastLeftVel = 0;
  double m_pastRightVel = 0;
  /** Creates a new CharacterizeDrivetrain. */
  public CharacterizeDrivetrain(DifferentialDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setControlMode(ControlMode.Voltage);
    m_targetVoltage = 0;
    m_pastLeftVel = 0;
    m_pastRightVel = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setLeftTarget(m_targetVoltage);
    m_drivetrain.setRightTarget(m_targetVoltage);
    m_targetVoltage += 0.01;
    System.out.print(m_targetVoltage);
    System.out.print(",");
    System.out.print(m_drivetrain.getLeftVelocity());
    System.out.print(",");
    System.out.print((m_drivetrain.getLeftVelocity() - m_pastLeftVel) / (20.0 / 1000.0));
    System.out.print(",");
    System.out.print(m_drivetrain.getRightVelocity());
    System.out.print(",");
    System.out.println((m_drivetrain.getRightVelocity() - m_pastRightVel) / (20.0 / 1000.0));

    m_pastLeftVel = m_drivetrain.getLeftVelocity();
    m_pastRightVel = m_drivetrain.getRightVelocity();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
