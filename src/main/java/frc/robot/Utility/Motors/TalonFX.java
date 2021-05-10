// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utility.Motors;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Add your docs here. */
public class TalonFX extends Motor {
    private WPI_TalonFX m_motor;

    public TalonFX(int canID){
        m_motor = new WPI_TalonFX(canID);

        if(m_motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor) == ErrorCode.SensorNotPresent){
            m_hasEncoder = false;
        }else{
            System.out.println("It has encoders");
            m_hasEncoder = true;
        }
        
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
                m_motor.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, m_target);
                break;
            
            case Voltage:
                m_motor.setVoltage(m_target);
                break;

            case Velocity:
                double feedForward = 0;//voltage
                if(m_isFeedforwardConfigured){
                    feedForward = m_feedforward.calculate(m_target, m_target - getVelocity());
                }
                m_motor.selectProfileSlot(0, 0);
                m_motor.set(com.ctre.phoenix.motorcontrol.ControlMode.Velocity, m_target, DemandType.ArbitraryFeedForward, feedForward / 12);
                break;
        }

        m_voltage = m_motor.getMotorOutputVoltage();
    }


    public double getPosition(){
        if(!m_hasEncoder){
            return Double.NaN;
        }
        return (double)m_motor.getSelectedSensorPosition() / 2048.0;
    }

    public double getVelocity(){
        if(!m_hasEncoder){
            return Double.NaN;
        }
        return (double)m_motor.getSelectedSensorVelocity() / 2048.0 / (100.0/1000.0);
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
        if(isInverted){
            m_motor.setInverted(InvertType.InvertMotorOutput);
            m_isInverted = true;
        }else{
            m_motor.setInverted(InvertType.None);
            m_isInverted = false;
        }     
    }

    public void resetEncoder(){
        m_motor.setSelectedSensorPosition(0);
    }


    public void enableBrakeMode(){
        m_motor.setNeutralMode(NeutralMode.Brake);
    }

    public void disableBrakeMode(){
        m_motor.setNeutralMode(NeutralMode.Coast);
    }


    public void setKP(double value){
        m_motor.config_kP(0, value);
    }

    public void setKI(double value){
        m_motor.config_kI(0, value);
    }

    public void setKD(double value){
        m_motor.config_kD(0, value);
    }
}
