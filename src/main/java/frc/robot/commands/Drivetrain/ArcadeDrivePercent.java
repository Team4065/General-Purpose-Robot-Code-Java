// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utility.Motors.Motor.ControlMode;
import frc.robot.subsystems.DifferentialDrivetrain2;

public class ArcadeDrivePercent extends CommandBase {
  DifferentialDrivetrain2 m_drivetrain;
  Joystick m_controller;
  /** Creates a new ArcadeDrivePercent. */
  public ArcadeDrivePercent(DifferentialDrivetrain2 drivetrain, Joystick controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setControlMode(ControlMode.PercentOutput);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -m_controller.getRawAxis(1);
    double rotation = -m_controller.getRawAxis(4);

    m_drivetrain.setLeftTarget(speed - rotation);
    m_drivetrain.setRightTarget(speed + rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setLeftTarget(0);
    m_drivetrain.setRightTarget(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
