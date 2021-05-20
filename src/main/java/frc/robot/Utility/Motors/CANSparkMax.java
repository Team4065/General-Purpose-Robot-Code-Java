// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility.Motors;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//To get the voltage outout of the CANSparkMax multiply the duty cycle by the bus voltage
//In other words do this:  m_motor.getAppliedOutput() * m_motor.getBusVoltage()

/**
 * This class is used to control the CANSparkMax.
 * It only supports brushless motors.
 */
public class CANSparkMax extends Motor{
    com.revrobotics.CANSparkMax m_motor;
    CANEncoder m_encoder;
    CANPIDController m_pidController;

    public CANSparkMax(int canID){
        m_motor = new com.revrobotics.CANSparkMax(canID, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        m_pidController = m_motor.getPIDController();
        m_hasEncoder = true;

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
                break;
            
            case Voltage:
                m_motor.setVoltage(m_target);
                break;

            case Velocity:
                double feedForward = 0;//voltage
                if(m_isFeedforwardConfigured){
                    feedForward = m_feedforward.calculate(m_target, m_target - getVelocity());
                }

                m_pidController.setReference(m_target, ControlType.kVelocity, 0, feedForward);
                break;
        }

        m_voltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
    }



    public double getPosition(){
        if(!m_hasEncoder){
            return Double.NaN;
        }
        return m_encoder.getPosition() * (m_isInverted ? -1.0 : 1.0);
    }

    public double getVelocity(){
        if(!m_hasEncoder){
            return Double.NaN;
        }
        return m_encoder.getVelocity() / 60.0  * (m_isInverted ? -1.0 : 1.0);
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
    }

    public void resetEncoder(){
        m_encoder.setPosition(0);
    }

    public void enableBrakeMode(){
        m_motor.setIdleMode(IdleMode.kBrake);
    }

    public void disableBrakeMode(){
        m_motor.setIdleMode(IdleMode.kCoast);
    }

    public void setKP(double value){
        m_pidController.setP(value, 0);
    }

    public void setKI(double value){
        m_pidController.setI(value, 0);
    }

    public void setKD(double value){
        m_pidController.setD(value, 0);
    }
}