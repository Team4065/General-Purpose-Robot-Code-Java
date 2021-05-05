// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility.Motors;

import java.util.Vector;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import frc.robot.Utility.Derivative;

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

    //The name explains what this does.
    public static void updateAllMotors(){
        for(Motor motor : allMotors){
            motor.update();
        }
    }

    protected abstract void update();//Updates the motor so that they actually do stuff.


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

    public abstract void follow(Motor master);//Sets the output of this motor to the voltage of the master. The follower spins in the same direction as the master, unless oppose master is true. SetInversion does not affect followers.
    public abstract void follow(Motor master, boolean opposeMaster);//Sets the output of this motor to the voltage of the master. The follower spins in the same direction as the master, unless oppose master is true. SetInversion does not affect followers.
    public abstract void setInversion(boolean isInverted);//Inverts the direction of the motor and the encoder. If the encoder reads negative with a positive output, the encoder will stil read negative when the motor is given a positive output after being inverted.
    public abstract void resetEncoder();//Sets the encoder position values to 0.
    public abstract void enableBrakeMode();//Causes the motor to apply back emf when output set to 0. (motor does not spin due to inertia)
    public abstract void disableBrakeMode();//prevents the motor from appling back emf when output set to 0. (motor spins due to inertia)

    //This configures the gains used to get an accuracy velocity from the motor
    public void configFeedforward(double ks, double kv){
        configFeedforward(ks, kv, 0);
    }

    //This configures the gains used to get an accuracy velocity from the motor
    public void configFeedforward(double ks, double kv, double ka){
        m_feedforward = new SimpleMotorFeedforward(ks, kv, ka);
        m_isFeedforwardConfigured = true;
    }


    public abstract void setKP(double kp);//Sets the propotional gain of the motor's velocity PID loop.
    public abstract void setKI(double ki);//Sets the integral gain of the motor's velocity PID loop.
    public abstract void setKD(double kd);//Sets the derivative gain of the motor's velocity PID loop.
}
