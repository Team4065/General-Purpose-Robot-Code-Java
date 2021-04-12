// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;
import java.util.Vector;
import java.util.function.Consumer;
import java.util.function.Supplier;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Utility.Regression;

public class FindFeedForwardGainsForVelocity extends CommandBase {
  private double m_value = 0;
  private Consumer<Double> m_set;
  private Supplier<Double> m_getVelocity;
  private Supplier<Double> m_getAcceleration;
  private double m_incrementSize;

  private Vector<Double> m_values = new Vector<Double>();
  private Vector<Double> m_velocities = new Vector<Double>();
  private Vector<Double> m_accelerations = new Vector<Double>();

  /** Creates a new FindFeedForwardGainsForVelocity. */
  public FindFeedForwardGainsForVelocity(Subsystem subsystem, Consumer<Double> set, Supplier<Double> getVelocity, Supplier<Double> getAcceleration, double incrementSize) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    m_set = set;
    m_getVelocity = getVelocity;
    m_getAcceleration = getAcceleration;
    m_incrementSize = incrementSize;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_value = 0;
    m_values = new Vector<Double>();
    m_velocities = new Vector<Double>();
    m_accelerations = new Vector<Double>();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_set.accept(m_value);
    m_values.add(m_value);
    m_velocities.add(m_getVelocity.get());
    m_accelerations.add(m_getAcceleration.get());
    m_value += m_incrementSize;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_set.accept(0.0);
    m_value = 0;
    
    Double[] values = Regression.findFeedForwardGainsForVelocity(m_values, m_velocities, m_accelerations);
    System.out.println("#");
    System.out.println("#");
    System.out.println("#");
    System.out.println(Arrays.toString(values));
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
