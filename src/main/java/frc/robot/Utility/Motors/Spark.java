// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility.Motors;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;

/** Add your docs here. */
public class Spark extends Motor {
    private edu.wpi.first.wpilibj.Spark m_motor;
    private PIDController m_PIDController;

    private Encoder m_encoder;

    public Spark(int pwmID){
        m_motor = new edu.wpi.first.wpilibj.Spark(pwmID);
        m_hasEncoder = false;
        Motor.allMotors.add(this);
    }

    public Spark(int pwmID, Encoder encoder){
        m_motor = new edu.wpi.first.wpilibj.Spark(pwmID);
        m_encoder = encoder;
        m_hasEncoder = true;
        m_PIDController = new PIDController(0, 0, 0);
        Motor.allMotors.add(this);
    }

    protected void update(){
        if(m_isSlave){
            setInversion((m_isOpposingMaster) ? !m_master.m_isInverted : m_master.m_isInverted);
            m_motor.setVoltage(m_master.m_voltage);
            return;
        }

        switch(m_controlMode){
            case PercentOutput:
                m_motor.set(m_target);
                m_voltage = m_target * 12;
                break;
            
            case Voltage:
                m_motor.setVoltage(m_target);
                m_voltage = m_target;
                break;

            case Velocity:
                double voltage = m_PIDController.calculate(getVelocity(), m_target);
                if(m_isFeedforwardConfigured){
                    voltage += m_feedforward.calculate(m_target, m_target - getVelocity());
                }
                
                m_motor.setVoltage(voltage);
                m_voltage = voltage;
                break;
        }
    }


    public double getPosition(){
        if(!m_hasEncoder){
            return Double.NaN;
        }
        return m_encoder.getDistance();
    }

    public double getVelocity(){
        if(!m_hasEncoder){
            return Double.NaN;
        }
        return m_encoder.getRate();
    }


    public void follow(Motor master){
        follow(master, false);
    }

    public void follow(Motor master, boolean opposeMaster){
        m_master = master;
        m_isSlave = true;
        m_isOpposingMaster = opposeMaster;
        m_isInverted = false;
    }

    public void setInversion(boolean isInverted){
        m_motor.setInverted(isInverted);
        m_isInverted = isInverted;   
        if(m_hasEncoder){
            m_encoder.setReverseDirection(isInverted);
        } 
    }

    public void resetEncoder(){
        if(m_hasEncoder){
            m_encoder.reset();
        }
    }

    public void enableBrakeMode(){
        System.out.println("The spark does not allow you to control break mode.");
    }

    public void disableBrakeMode(){
        System.out.println("The spark does not allow you to control break mode.");
    }


    public void setKP(double value){
        if(m_hasEncoder){
            m_PIDController.setP(value);
        }
    }

    public void setKI(double value){
        if(m_hasEncoder){
            m_PIDController.setI(value);
        }
    }

    public void setKD(double value){
        if(m_hasEncoder){
            m_PIDController.setD(value);
        }
    }

}
