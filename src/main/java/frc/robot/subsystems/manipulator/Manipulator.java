/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.manipulator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {
  /**
   * Creates a new Manipulator.
   */
  public Manipulator() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void gotoInitialRobotState(){}

  public void gotoInitialTeleopState(){}

  public void gotoInitialAutonomusState(){}

}
