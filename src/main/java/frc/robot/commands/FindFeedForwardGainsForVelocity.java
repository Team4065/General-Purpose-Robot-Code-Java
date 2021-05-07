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
  private double m_incrementSize;

  private Vector<Double> m_values = new Vector<Double>();
  private Vector<Double> m_velocities = new Vector<Double>();

  /** Creates a new FindFeedForwardGainsForVelocity. */
  public FindFeedForwardGainsForVelocity(Subsystem subsystem, Consumer<Double> set, Supplier<Double> getVelocity, Supplier<Double> getAcceleration, double incrementSize) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    m_set = set;
    m_getVelocity = getVelocity;
    m_incrementSize = incrementSize;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_set.accept(0.0);
    m_value = 0;
    m_values = new Vector<Double>();
    m_velocities = new Vector<Double>();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_set.accept(m_value);
    if(Math.abs(m_getVelocity.get()) > 0.000000001){
      m_values.add(m_value);
      m_velocities.add(m_getVelocity.get());
      
      System.out.printf("%.8f", m_getVelocity.get());
      System.out.print(",");
      System.out.printf("%.8f", m_value);
      System.out.println("");
      
    }
    
    m_value += m_incrementSize;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_set.accept(0.0);
    m_value = 0;
    Double[] values = Regression.findFeedForwardGainsForVelocity(m_values, m_velocities, findSlope());
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


  private double findSlope(){
    double totalSlope = 0;
    int counter = 0;
    for(int i = 0; i < m_values.size(); ++i){
      for(int j = 0; j < m_values.size(); ++j){
        if(i != j){
          double slope = (m_values.get(j) - m_values.get(i)) / (m_velocities.get(j) - m_velocities.get(i));
          if(!Double.isNaN(slope) && !Double.isInfinite(slope)){
            totalSlope += slope;
            ++counter;
          }
            
        }
      }
    }

    return totalSlope / (double)counter;
  }
}
