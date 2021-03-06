// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.Vector;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utility.Motors.Motor.ControlMode;
import frc.robot.subsystems.DifferentialDrivetrain2;

public class RecordOutput extends CommandBase {
  DifferentialDrivetrain2 m_drivetrain;
  Joystick m_controller;
  double m_maxSpeed;
  double m_maxRotationalSpeed;

  Vector<Double> m_leftRecording;
  Vector<Double> m_rightRecording;

  /** Creates a new RecordOutput. */
  public RecordOutput(DifferentialDrivetrain2 drivetrain, Joystick controller, double maxSpeed, double maxRotationalSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_controller = controller;
    m_maxSpeed = maxSpeed;
    m_maxRotationalSpeed = maxRotationalSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setControlMode(ControlMode.Velocity);
    m_leftRecording = new Vector<Double>();
    m_rightRecording = new Vector<Double>();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = -m_controller.getRawAxis(1);
    double rotation = -m_controller.getRawAxis(4);

    if(Math.abs(speed) < 0.05)
      speed = 0;
    if(Math.abs(rotation) < 0.05)
      rotation = 0;

    speed *= m_maxSpeed;
    rotation *= Math.toRadians(m_maxRotationalSpeed) * m_drivetrain.getTrackWidth() / 2;

    m_drivetrain.setLeftTarget(speed - rotation);
    m_drivetrain.setRightTarget(speed + rotation);

    m_leftRecording.add(speed - rotation);
    m_rightRecording.add(speed + rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setControlMode(ControlMode.PercentOutput);
    m_drivetrain.setLeftTarget(0);
    m_drivetrain.setRightTarget(0);

    System.out.println("#");
    System.out.println("#");
    System.out.println("#");
    System.out.println("#");
    System.out.println("# Left");

    System.out.println(m_leftRecording.toString());

    System.out.println("# Left");
    System.out.println("#");
    System.out.println("#");
    System.out.println("#");
    System.out.println("# Right");

    System.out.println(m_rightRecording.toString());

    System.out.println("# Right");
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
