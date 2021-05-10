// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utility.Motors.TalonFX;
import frc.robot.Utility.Motors.Motor.ControlMode;

public class Lift extends SubsystemBase {
  TalonFX m_leftMotor = new TalonFX(0);
  TalonFX m_rightMotor = new TalonFX(0);

  /** Creates a new Lift. */
  public Lift() {
    m_rightMotor.follow(m_leftMotor, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void forward(){
    m_leftMotor.set(ControlMode.PercentOutput, 1);
  }

  public void backward(){
    m_leftMotor.set(ControlMode.PercentOutput, -1);
  }

  public void stop(){
    m_leftMotor.set(ControlMode.PercentOutput, 0);
  }
}
