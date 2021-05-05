// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility.Motors;

import java.util.Vector;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Utility.Derivative;
import frc.robot.Utility.Motor.ControlMode;

/** Add your docs here. */
public abstract class Motor {
    protected static Vector<Motor> allMotors = new Vector<Motor>();

    public enum ControlMode {
        PercentOutput, Voltage, Velocity
    };

    protected ControlMode m_controlMode = ControlMode.PercentOutput;
    protected double m_target = 0;
    protected double m_voltage = 0;

    protected boolean m_hasEncoder = false;

    protected Derivative m_acceleration = new Derivative(()->{return getVelocity();});

    protected Motor m_master;
    protected boolean m_isSlave = false;
    protected boolean m_isOpposingMaster = false;


    protected SimpleMotorFeedforward m_feedforward;
    protected boolean m_isFeedforwardConfigured = false;

    protected boolean m_isInverted = false;


    public static void updateAllMotors(){
        for(Motor motor : allMotors){
            motor.update();
        }
    }

    protected abstract void update();


    public void set(ControlMode controlMode, double value){
        m_target = value;
        m_controlMode = controlMode;
        if(controlMode == ControlMode.Velocity && !m_hasEncoder){
            m_controlMode = ControlMode.PercentOutput;
            m_target = 0;
            System.out.print("Must have encoder to use velocity control.");
        }
            
    }

    /**
     * 
     * @return Rotations
     */
    public abstract double getPosition();
    /**
     * 
     * @return Rotations per second
     */
    public abstract double getVelocity();
    /**
     * 
     * @return rotations per second squared
     */
    public double getAcceleration(){
        if(m_hasEncoder){
            return m_acceleration.get();
        }else{
            return Double.NaN;
        }
    }

    public abstract void follow(Motor master);
    public abstract void follow(Motor master, boolean opposeMaster);
    public abstract void setInversion(boolean isInverted);
    public abstract void resetEncoder();
    public abstract void enableBrakeMode();
    public abstract void disableBrakeMode();

    public void configFeedforward(double ks, double kv){
        configFeedforward(ks, kv, 0);
    }

    public void configFeedforward(double ks, double kv, double ka){
        m_feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        m_isFeedforwardConfigured = true;
    }


    public abstract void setKP(double kp);
    public abstract void setKI(double ki);
    public abstract void setKD(double kd);
}
