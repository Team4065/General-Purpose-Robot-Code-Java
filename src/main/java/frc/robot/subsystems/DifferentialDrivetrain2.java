// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utility.Gyro;
import frc.robot.Utility.Motors.Motor;
import frc.robot.Utility.Motors.Motor.ControlMode;
import frc.robot.commands.FindFeedForwardGainsForVelocity;

public class DifferentialDrivetrain2 extends SubsystemBase {

  HashMap<String, NetworkTableEntry> m_spyTab = new HashMap<String, NetworkTableEntry>();

  private Motor m_leftMaster;
  private Motor[] m_leftSlaves;

  private Motor m_rightMaster;
  private Motor[] m_rightSlaves;

  protected DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(Gyro.getRotation2d());

  protected double m_wheelDiameter;
  protected double m_gearRatio;//1 rotation of wheel / spins of encoder
  protected boolean m_areEncodersInverted;
  protected double m_trackWidth;
  

  protected double m_leftTarget = 0;
  protected double m_rightTarget = 0;
  protected boolean m_isInverted;
  protected ControlMode m_controlMode = ControlMode.PercentOutput;

  public FindFeedForwardGainsForVelocity findLeftFeedForwardGains = new FindFeedForwardGainsForVelocity(this,
      (Double voltage)->{
        setControlMode(ControlMode.Voltage);
        setLeftTarget(voltage);
        setRightTarget(voltage);
      }, 
      ()->{
        return m_leftMaster.getVelocity();
      }, 
      ()->{
        return m_leftMaster.getAcceleration();
      }, 0.005);
    
  public FindFeedForwardGainsForVelocity findRightFeedForwardGains = new FindFeedForwardGainsForVelocity(this,
      (Double voltage)->{
        setControlMode(ControlMode.Voltage);
        setLeftTarget(voltage);
        setRightTarget(voltage);
      }, 
      ()->{
        return m_rightMaster.getVelocity();
      }, 
      ()->{
        return m_rightMaster.getAcceleration();
      }, 0.005);

  /** Creates a new DifferentialDrivetrain2. */
  public DifferentialDrivetrain2(Motor leftMaster, Motor rightMaster, Motor[] leftSlaves, Motor[] rightSlaves, double wheelDiameter, double gearRatio, double trackWidth, boolean isInverted, boolean areEncodersInverted) {
    m_leftMaster = leftMaster;
    m_leftSlaves = leftSlaves;
    for(Motor slave : m_leftSlaves){
      slave.follow(m_leftMaster);
    }

    m_rightMaster = rightMaster;
    m_rightSlaves = rightSlaves;
    for(Motor slave : m_rightSlaves){
      slave.follow(m_rightMaster);
    }

    m_rightMaster.setInversion(true);

    m_wheelDiameter = wheelDiameter;
    m_gearRatio = gearRatio;
    m_areEncodersInverted = areEncodersInverted;
    m_isInverted = isInverted;
    m_trackWidth = trackWidth;

    resetPose();
  }

  @Override
  public void periodic() {
    double leftTarget = (m_isInverted ? -1 : 1) * m_leftTarget;
    double rightTarget = (m_isInverted ? -1 : 1) * m_rightTarget;

    if(m_controlMode == ControlMode.Velocity){
      m_leftMaster.set(ControlMode.Velocity, leftTarget / 2 / Math.PI / (m_wheelDiameter / 2) / m_gearRatio * ((m_areEncodersInverted) ? -1 : 1));
      m_rightMaster.set(ControlMode.Velocity, rightTarget / 2 / Math.PI / (m_wheelDiameter / 2) / m_gearRatio * ((m_areEncodersInverted) ? -1 : 1));
    }else{
      m_leftMaster.set(m_controlMode, leftTarget);
      m_rightMaster.set(m_controlMode, rightTarget);
    }

    m_odometry.update(Gyro.getRotation2d(), getLeftPosition(), getRightPosition());

    /*
    System.out.print(getPose().getX());
    System.out.print(",");
    System.out.println(getPose().getY());
    */
  }

  public void setControlMode(ControlMode controlMode){
    m_controlMode = controlMode;
  }

  public void setLeftTarget(double value){
    m_leftTarget = value;
  }

  public void setRightTarget(double value){
    m_rightTarget = value;
  }

  public void setInversion(boolean isInverted){
    m_isInverted = isInverted;
  }


  public double getLeftPosition(){
    return ((m_areEncodersInverted) ? -1 : 1) * 2 * Math.PI * m_leftMaster.getPosition() * m_gearRatio * (m_wheelDiameter / 2);
  }

  public double getRightPosition(){
    return ((m_areEncodersInverted) ? -1 : 1) * 2 * Math.PI * m_rightMaster.getPosition() * m_gearRatio * (m_wheelDiameter / 2);
  }

  public double getLeftVelocity(){
    return ((m_areEncodersInverted) ? -1 : 1) * 2 * Math.PI * m_leftMaster.getVelocity() * m_gearRatio * (m_wheelDiameter / 2);
  }

  public double getRightVelocity(){
    return ((m_areEncodersInverted) ? -1 : 1) * 2 * Math.PI * m_rightMaster.getVelocity() * m_gearRatio * (m_wheelDiameter / 2);
  }

  public double getLeftAcceleration(){
    return ((m_areEncodersInverted) ? -1 : 1) * 2 * Math.PI * m_leftMaster.getAcceleration() * m_gearRatio * (m_wheelDiameter / 2);
  }

  public double getRightAcceleration(){
    return ((m_areEncodersInverted) ? -1 : 1) * 2 * Math.PI * m_rightMaster.getAcceleration() * m_gearRatio * (m_wheelDiameter / 2);
  }


  public void resetEncoders(){
    m_leftMaster.resetEncoder();
    m_rightMaster.resetEncoder();
  }


  public void configLeftFeedForward(double ks, double kv){
    m_leftMaster.configFeedforward(ks, kv);
  }

  public void configRightFeedForward(double ks, double kv){
    m_rightMaster.configFeedforward(ks, kv);
  }


  public void configLeftPID(double kp, double ki, double kd){
    m_leftMaster.setKP(kp);
    m_leftMaster.setKI(ki);
    m_leftMaster.setKD(kd);
  }

  public void configRightPID(double kp, double ki, double kd){
    m_rightMaster.setKP(kp);
    m_rightMaster.setKI(ki);
    m_rightMaster.setKD(kd);
  }


  public void enableBrakes(){
    m_leftMaster.enableBrakeMode();
    m_rightMaster.enableBrakeMode();
  }

  public void disableBrakes(){
    m_leftMaster.disableBrakeMode();
    m_rightMaster.disableBrakeMode();
  }


  public double getTrackWidth(){
    return m_trackWidth;
  }


  //Ramsete functions
  public void resetPose(){
    resetEncoders();
    m_odometry.resetPosition(new Pose2d(), Gyro.getRotation2d());
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public void setTankDriveVelocity(double left, double right){
    setControlMode(ControlMode.Velocity);
    setLeftTarget(left);
    setRightTarget(right);
  }
  
  public void resetOdometry(Pose2d pose){
    resetEncoders();
    m_odometry.resetPosition(pose, Gyro.getRotation2d());
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
