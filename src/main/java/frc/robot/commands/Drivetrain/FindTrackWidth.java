// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utility.Gyro;
import frc.robot.Utility.Motors.Motor.ControlMode;
import frc.robot.subsystems.DifferentialDrivetrain2;

public class FindTrackWidth extends CommandBase {
  DifferentialDrivetrain2 m_drivetrain;

  /** Creates a new FindTrackWidth. */
  public FindTrackWidth(DifferentialDrivetrain2 drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetEncoders();
    Gyro.reset();
    m_drivetrain.setControlMode(ControlMode.PercentOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.setLeftTarget(0.1);
    m_drivetrain.setRightTarget(-0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setLeftTarget(0);
    m_drivetrain.setRightTarget(0);

    double rotations = (Gyro.getAngle() / 360);
    double distance = (Math.abs(m_drivetrain.getLeftPosition()) + Math.abs(m_drivetrain.getRightPosition())) / 2;

    double radius = distance / (rotations * 2.0 * Math.PI);
    System.out.println("#");
    System.out.println("#");
    System.out.println("#");
    System.out.println("#");
    System.out.println("#");
    System.out.println(Math.abs(radius * 2));
    System.out.println("#");
    System.out.println("#");
    System.out.println("#");
    System.out.println("#");
    System.out.println("#");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
