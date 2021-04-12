// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utility.Gyro;
import frc.robot.Utility.Motor;

public class DifferentialDrivetrain extends SubsystemBase {
  public enum ControlMode{
    PercentOutput,
    Voltage,
    Velocity
  };

  HashMap<String, NetworkTableEntry> m_spyTab = new HashMap<String, NetworkTableEntry>();

  Motor m_leftMaster, m_rightMaster;
  Motor[] m_leftSlaves, m_rightSlaves;

  protected double m_leftTarget;
  protected double m_rightTarget;
  protected double m_theoreticalTrackWidth;
  protected boolean m_isTheoreticalTrackWidthConfigured = false;
  protected SimpleMotorFeedforward m_leftFeedforward, m_rightFeedforward;
  protected boolean m_isFeedforwardConfigured = false;
  protected ControlMode m_controlMode = ControlMode.PercentOutput;
  protected boolean m_isInverted = false;

  protected SimpleMotorFeedforward m_rotationFeedforward;
  protected boolean m_isRotationFeedforwardConfigured = false;

  protected DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(new Rotation2d());
  protected double m_wheelDiameter;
  protected double m_gearRatio;//1 rotation of wheel / spins of encoder
  protected boolean m_areEncodersInverted;

  /**
   * Creates a new Differential Drivetrain
   * @param wheelDiameter
   * @param gearRatio 1 spin of wheel / spins of encoder
   * @param isInverted Is the direction of the drivetrain backwards?
   * @param areEncodcersInverted Do the encoders need to be inverted to that forward reads as positive? (Inverting the direction of the drivetrain will not solve this issue.)
   * @param leftMaster
   * @param rightMaster
   * @param leftSlaves
   * @param rightSlaves
   */
  public DifferentialDrivetrain(double wheelDiameter, double gearRatio, boolean isInverted, boolean areEncodersInverted, Motor leftMaster, Motor rightMaster, Motor[] leftSlaves, Motor[] rightSlaves) {
    m_wheelDiameter = wheelDiameter;
    m_gearRatio  = gearRatio;

    m_leftMaster = leftMaster;
    m_rightMaster = rightMaster;
    m_leftSlaves = leftSlaves;
    m_rightSlaves = rightSlaves;

    m_leftMaster.enableBrakeMode(true);
    m_rightMaster.enableBrakeMode(true);

    setInversion(isInverted);
    m_areEncodersInverted = areEncodersInverted;
    
    m_odometry.resetPosition(new Pose2d(), new Rotation2d());

    for(Motor slave : m_leftSlaves){
      slave.follow(m_leftMaster);
    }

    for(Motor slave : m_rightSlaves){
      slave.follow(m_rightMaster);
    }

    if(Constants.IS_SPY_ENABLED){
      makeSpy();
    }
  }

  @Override
  public void periodic() {
    m_odometry.update(Gyro.getRotation2d(), getLeftPosition(), getRightPosition());

    if(Constants.IS_SPY_ENABLED){
      reportSpy();
    }

    switch(m_controlMode){
      case PercentOutput:
        setLeftPercent(m_leftTarget);
        setRightPercent(m_rightTarget);
        break;

      case Voltage:
        setLeftVoltage(m_leftTarget);
        setRightVoltage(m_rightTarget);
        break;

      case Velocity:
        setLeftVelocity(m_leftTarget);
        setRightVelocity(m_rightTarget);
        break;

      default:
        setLeftPercent(0);
        setRightPercent(0);
        break;
    }
  
  }


    /**
   * Configures the feedforward.
   * This must be done before velocity control may be used.
   * Values can be obtained from robot characterization.
   * @param kS
   * @param kV
   * @param kA
   */
  public void configFeedforward(double kS, double kV, double kA){
    m_isFeedforwardConfigured = true;
    m_leftFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
    m_rightFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }
  
  /**
   * Same as configFeedforward, but it configures the left and right sides separately.
   * Values need to be obtained manually.
   * @param left_kS
   * @param left_kV
   * @param left_kA
   * @param right_kS
   * @param right_kV
   * @param right_kA
   */
  public void configFeedforwardSided(double left_kS, double left_kV, double left_kA, double right_kS, double right_kV, double right_kA){
    m_isFeedforwardConfigured = true;

    m_leftFeedforward = new SimpleMotorFeedforward(left_kS, left_kV, left_kA);
    m_rightFeedforward = new SimpleMotorFeedforward(right_kS, right_kV, right_kA);
  }

  public void configRotationFeedForward(double kS, double kV, double kA){
    m_isRotationFeedforwardConfigured = true;
    m_rotationFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  public void configTheoreticalTrackWidth(double width){
    m_theoreticalTrackWidth = width;
    m_isTheoreticalTrackWidthConfigured = true;
  }


  public void setControlMode(ControlMode mode){
    m_controlMode = mode;
  }

  public void setLeftTarget(double target){
    m_leftTarget = target;
  }

  public void setRightTarget(double target){
    m_rightTarget = target;
  }

  public void setArcadeDrive(double speed, double rotation){
    if(m_isRotationFeedforwardConfigured && m_controlMode == ControlMode.Velocity){
      double acceleration = rotation - Gyro.getRate();
      if(Math.abs(acceleration) < 3)
        acceleration = 0;
      
      double rotationVelocity = m_rotationFeedforward.calculate(rotation, acceleration);
      setLeftTarget(speed + rotationVelocity);
      setRightTarget(speed - rotationVelocity);
    }else{
      setLeftTarget(speed + rotation);
      setRightTarget(speed - rotation);
    }
  }

  public void setArcadeDrive2(double speed, double rotation){
    if(m_isTheoreticalTrackWidthConfigured && m_controlMode == ControlMode.Velocity){
      double leftSpeed = Math.toRadians(rotation) * (speed - m_theoreticalTrackWidth);
      double rightSpeed = Math.toRadians(rotation) * (speed + m_theoreticalTrackWidth);

      setLeftTarget(leftSpeed);
      setRightTarget(rightSpeed);
    }else{
      setLeftTarget(speed + rotation);
      setRightTarget(speed - rotation);
    }
  }

  public void setTankDriveVolts(double left, double right){
    setControlMode(ControlMode.Voltage);
    setLeftTarget(left);
    setRightTarget(right);
  }



  protected void setLeftPercent(double percent){
    m_leftMaster.set(frc.robot.Utility.Motor.ControlMode.PercentOutput, percent);
  }
  
  protected void setRightPercent(double percent){
    m_rightMaster.set(frc.robot.Utility.Motor.ControlMode.PercentOutput, percent);
  }

  protected void setLeftVoltage(double voltage){
    m_leftMaster.set(frc.robot.Utility.Motor.ControlMode.Voltage, voltage);
  }

  protected void setRightVoltage(double voltage){
    m_rightMaster.set(frc.robot.Utility.Motor.ControlMode.Voltage, voltage);
  }

  protected void setLeftVelocity(double velocity){
    if(m_isFeedforwardConfigured){
      double voltage = m_leftFeedforward.calculate(velocity, velocity - getLeftVelocity());
      m_leftMaster.set(frc.robot.Utility.Motor.ControlMode.Voltage, voltage);
    }else{
      setLeftPercent(0);
    }
  }

  protected void setRightVelocity(double velocity){
    if(m_isFeedforwardConfigured){
      double voltage = m_rightFeedforward.calculate(velocity, velocity - getRightVelocity());
      m_rightMaster.set(frc.robot.Utility.Motor.ControlMode.Voltage, voltage);
    }else{
      setRightPercent(0);
    }
  }

  /**
   * 
   * @return The acceleration of the left side in meters per second squared
   */
  public double getLeftAcceleration(){
    return ((m_areEncodersInverted) ? -1 : 1) * m_leftMaster.getAcceleration() * m_gearRatio * m_wheelDiameter * Math.PI;
  }

  /**
   * 
   * @return The acceleration of the right side in meters per second squared
   */
  public double getRightAcceleration(){
    return ((m_areEncodersInverted) ? -1 : 1) * m_rightMaster.getAcceleration() * m_gearRatio * m_wheelDiameter * Math.PI;
  }

  /**
   * @return The velocity of the left side in meters per second
   */
  public double getLeftVelocity(){
    return ((m_areEncodersInverted) ? -1 : 1) * m_leftMaster.getVelocity() * m_gearRatio * m_wheelDiameter * Math.PI;
  }

  /**
   * @return The velocity of the right side in meters per second
   */
  public double getRightVelocity(){
    return ((m_areEncodersInverted) ? -1 : 1) * m_rightMaster.getVelocity() * m_gearRatio * m_wheelDiameter * Math.PI;
  }

  /**
   * @return The distance the left side has traveled in meters.
   */
  public double getLeftPosition(){
    return ((m_areEncodersInverted) ? -1 : 1) * m_leftMaster.getPosition() * m_gearRatio * m_wheelDiameter * Math.PI;
  }

  /**
   * @return The distance the right side has traveled in meters.
   */
  public double getRightPosition(){
    return ((m_areEncodersInverted) ? -1 : 1) * m_rightMaster.getPosition() * m_gearRatio * m_wheelDiameter * Math.PI;
  }

  public void resetEncoders(){
    m_leftMaster.resetEncoder();
    m_rightMaster.resetEncoder();
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public void setInversion(boolean isInverted){
    m_leftMaster.setInverted(isInverted);
    m_rightMaster.setInverted(!isInverted);
  }



  protected void makeSpy(){
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    m_spyTab.put("Left Velocity", tab.add("Left Velocity", 0).getEntry());
    m_spyTab.put("Right Velocity", tab.add("Right Velocity", 0).getEntry());
    m_spyTab.put("X Position", tab.add("X Position", 0).getEntry());
    m_spyTab.put("Y Position", tab.add("Y Position", 0).getEntry());
    m_spyTab.put("Heading", tab.add("Heading", 0).getEntry());
    m_spyTab.put("Rotational Velocity", tab.add("Rotate Vel", 0).getEntry());

    m_spyTab.put("Left Position", tab.add("Left Pos", 0).getEntry());
    m_spyTab.put("Right Position", tab.add("Right Pos", 0).getEntry());

    m_spyTab.put("Left Target", tab.add("Left Target", 0).getEntry());
    m_spyTab.put("Right Target", tab.add("Right Target", 0).getEntry());
  }

  protected void reportSpy(){
    m_spyTab.get("Left Velocity").setDouble(getLeftVelocity());
    m_spyTab.get("Right Velocity").setDouble(getRightVelocity());

    m_spyTab.get("X Position").setDouble(m_odometry.getPoseMeters().getX());
    m_spyTab.get("Y Position").setDouble(m_odometry.getPoseMeters().getY());
    m_spyTab.get("Heading").setDouble(m_odometry.getPoseMeters().getRotation().getDegrees());
    m_spyTab.get("Rotational Velocity").setDouble(Gyro.getRate());

    m_spyTab.get("Left Position").setDouble(getLeftPosition());
    m_spyTab.get("Right Position").setDouble(getRightPosition());

    m_spyTab.get("Left Target").setDouble(m_leftTarget);
    m_spyTab.get("Right Target").setDouble(m_rightTarget);
  }
}
