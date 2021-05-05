// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.Utility;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

/** Add your docs here. */
public class Motor {
    public enum ControlMode {
        PercentOutput, Voltage, Velocity
    };

    public enum MotorType{
        TalonSRX,
        TalonFX,
        VictorSPX,
        CANSparkMax,
        Spark
    };

    WPI_TalonSRX m_talonSRX;
    WPI_TalonFX m_talonFX;
    WPI_VictorSPX m_victorSPX;
    CANSparkMax m_canSparkMax;
    CANEncoder m_canEncoder;


    Spark m_spark;
    Encoder m_encoder;

    MotorType m_motorType;
    boolean m_isFeedforwardConfigured = false;
    SimpleMotorFeedforward m_feedforward;

    private Derivative m_acceleration = new Derivative(()->getVelocity());

    public Motor(int id, MotorType motorType) {
        m_motorType = motorType;
        switch(m_motorType){
            case TalonSRX:
                m_talonSRX = new WPI_TalonSRX(id);
                m_talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
                m_talonSRX.setSelectedSensorPosition(0);
                break;

            case TalonFX:
                m_talonFX = new WPI_TalonFX(id);
                m_talonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                m_talonFX.setSelectedSensorPosition(0);
                break;

            case VictorSPX:
                m_victorSPX = new WPI_VictorSPX(id);
                break;

            case CANSparkMax:
                m_canSparkMax = new CANSparkMax(id, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
                m_canEncoder = m_canSparkMax.getEncoder();
                m_canEncoder.setPosition(0);
                break;

            case Spark:
                m_spark = new Spark(id);
                break;
        }
    }

    public Motor(int id, MotorType motorType, Encoder encoder) {
        m_motorType = motorType;
        m_encoder = encoder;

        switch(m_motorType){
            case TalonSRX:
                m_talonSRX = new WPI_TalonSRX(id);
                m_talonSRX.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
                m_talonSRX.setSelectedSensorPosition(0);
                break;

            case TalonFX:
                m_talonFX = new WPI_TalonFX(id);
                m_talonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
                m_talonFX.setSelectedSensorPosition(0);
                break;

            case VictorSPX:
                m_victorSPX = new WPI_VictorSPX(id);
                break;

            case CANSparkMax:
                m_canSparkMax = new CANSparkMax(id, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
                m_canEncoder = m_canSparkMax.getEncoder();
                m_canEncoder.setPosition(0);
                break;

            case Spark:
                m_spark = new Spark(id);
                break;
        }
    }

    /**
     * Sets the motor output.
     * @param controlMode Percent output, voltage, or velocity (velocity requires feedforward to be configured)
     * @param value The value to output. In the same units as the control mode.
     */
    public void set(ControlMode controlMode, double value) {
        if(!m_isFeedforwardConfigured && controlMode == ControlMode.Velocity){
            controlMode = ControlMode.PercentOutput;
            value = 0;
            System.out.println("You must configure feedforward to use Velocity control.");
        }

        switch(m_motorType){
            case TalonSRX:
                switch (controlMode) {
                    case PercentOutput:
                        m_talonSRX.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
                        break;
                    case Voltage:
                        m_talonSRX.setVoltage(value);
                        break;
                    case Velocity:
                        m_talonSRX.setVoltage(m_feedforward.calculate(value, value - getVelocity()));
                        break;
                    default:
                        m_talonSRX.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
                        break;
                }
                break;

            case TalonFX:
                switch (controlMode) {
                    case PercentOutput:
                        m_talonFX.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
                        break;
                    case Voltage:
                        m_talonFX.setVoltage(value);
                        break;
                    case Velocity:
                        m_talonFX.setVoltage(m_feedforward.calculate(value, value - getVelocity()));
                        break;
                    default:
                        m_talonFX.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
                        break;
                }
                break;
            
            case VictorSPX:
                switch (controlMode) {
                    case PercentOutput:
                        m_victorSPX.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
                        break;
                    case Voltage:
                        m_victorSPX.setVoltage(value);
                        break;
                    case Velocity:
                        m_victorSPX.setVoltage(m_feedforward.calculate(value, value - getVelocity()));
                        break;
                    default:
                        m_victorSPX.set(com.ctre.phoenix.motorcontrol.ControlMode.PercentOutput, value);
                        break;
                }
                break;
            
            case CANSparkMax:
                switch (controlMode) {
                    case PercentOutput:
                        m_canSparkMax.set(value);
                        break;
                    case Voltage:
                        m_canSparkMax.setVoltage(value);
                        break;
                    case Velocity:
                        m_canSparkMax.setVoltage(m_feedforward.calculate(value, value - getVelocity()));
                        break;
                    default:
                        m_canSparkMax.set(value);
                        break;
                }
                break;
            
            case Spark:
                switch(controlMode){
                    case PercentOutput:
                        m_spark.set(value);
                        break;

                    case Voltage:
                        m_spark.setVoltage(value);
                        break;
                    
                    case Velocity:
                        m_spark.setVoltage(m_feedforward.calculate(value, value - getVelocity()));
                        break;

                    default:
                        m_spark.set(value);
                        break;
                }
                break;
            
        }

    }

    /**
     * 
     * @return RPS^2 of motor
     */
    public double getAcceleration(){
        return m_acceleration.get();
    }

    /**
     * 
     * @return RPS of motor
     */
    public double getVelocity() {
        switch(m_motorType){
            case TalonSRX:
                return (double) m_talonSRX.getSelectedSensorVelocity() / 4096.0 * 10.0;

            case TalonFX:
                return (double) m_talonFX.getSelectedSensorVelocity() / 2048.0 * 10.0;

            case VictorSPX:
                if (m_encoder != null)
                    return m_encoder.getRate();
                else
                    return Double.NaN;

            case CANSparkMax:
                return m_canEncoder.getVelocity() / 60.0;
            
            case Spark:
                if (m_encoder != null)
                    return m_encoder.getRate();
                else
                    return Double.NaN;

            default:
                return Double.NaN;
        }
    }

    /**
     * 
     * @return Revolutions of motor
     */
    public double getPosition() {
        switch(m_motorType){
            case TalonSRX:
                return (double) m_talonSRX.getSelectedSensorPosition() / 4096.0;

            case TalonFX:
                return (double) m_talonFX.getSelectedSensorPosition() / 2048.0;

            case VictorSPX:
                if (m_encoder != null)
                    return m_encoder.getDistance();
                else
                    return Double.NaN;

            case CANSparkMax:
                return m_canEncoder.getPosition();
            
            case Spark:
                if (m_encoder != null)
                    return m_encoder.getDistance();
                else
                    return Double.NaN;
            
            default:
                return Double.NaN;

        }
    }

    

    /**
     * 
     * @param master Motor to mimic the voltage of. The motor will spin in the same direction.
     * @param opposeMaster Should the voltage be inverted. If true the motor will spin in the opposite direction.
     */
    public void follow(Motor master, boolean opposeMaster) {
        boolean isSelfCTRE = m_motorType == MotorType.TalonSRX || m_motorType == MotorType.TalonFX || m_motorType == MotorType.VictorSPX;
        boolean isMasterCTRE = master.m_motorType == MotorType.TalonSRX || master.m_motorType == MotorType.TalonFX || master.m_motorType == MotorType.VictorSPX;

        if (isSelfCTRE && isMasterCTRE) {
            boolean isMasterTalonSRX = master.m_motorType == MotorType.TalonSRX;
            boolean isMasterTalonFX = master.m_motorType == MotorType.TalonFX;

            var masterMotor = (isMasterTalonSRX) ? master.m_talonSRX
                    : ((isMasterTalonFX) ? master.m_talonFX : master.m_victorSPX);

            if (m_motorType == MotorType.TalonSRX) {
                m_talonSRX.follow(masterMotor);
                m_talonSRX.setInverted((opposeMaster) ? InvertType.OpposeMaster : InvertType.FollowMaster);
            }

            if (m_motorType == MotorType.TalonFX) {
                m_talonFX.follow(masterMotor);
                m_talonFX.setInverted((opposeMaster) ? InvertType.OpposeMaster : InvertType.FollowMaster);
            }

            if (m_motorType == MotorType.VictorSPX) {
                m_victorSPX.follow(masterMotor);
                m_victorSPX.setInverted((opposeMaster) ? InvertType.OpposeMaster : InvertType.FollowMaster);
            }
            return;
        }

        if (m_motorType == MotorType.CANSparkMax && master.m_motorType == MotorType.CANSparkMax) {
            m_canSparkMax.follow(master.m_canSparkMax, opposeMaster);
            return;
        }

        try {
            throw new Exception("Cannot follow that motor type.");
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * 
     * @param master Motor to mimic the voltage of. The motor will spin in the same direction.
     */
    public void follow(Motor master){
        follow(master, false);
    }

    /**
     * Inverts the direction of the motor.
     * Does nothing if the motor is following another motor.
     * @param isInverted 
     */
    public void setInverted(boolean isInverted){
        if (m_motorType == MotorType.TalonSRX) {
            if(m_talonSRX.getControlMode() != com.ctre.phoenix.motorcontrol.ControlMode.Follower)
                m_talonSRX.setInverted(isInverted);
        }

        if (m_motorType == MotorType.TalonFX) {
            if(m_talonFX.getControlMode() != com.ctre.phoenix.motorcontrol.ControlMode.Follower)
                m_talonFX.setInverted(isInverted);
        }

        if (m_motorType == MotorType.VictorSPX) {
            if(m_victorSPX.getControlMode() != com.ctre.phoenix.motorcontrol.ControlMode.Follower)
                m_victorSPX.setInverted(isInverted);
        }

        if (m_motorType == MotorType.CANSparkMax) {
            if(!m_canSparkMax.isFollower())
                m_canSparkMax.setInverted(isInverted);
        }

        if (m_motorType == MotorType.Spark){
            m_spark.setInverted(isInverted);
        }
    }

    public void resetEncoder(){
        if (m_motorType == MotorType.TalonSRX) {
            m_talonSRX.setSelectedSensorPosition(0);
        }

        if (m_motorType == MotorType.TalonFX) {
            m_talonFX.setSelectedSensorPosition(0);
        }

        if (m_motorType == MotorType.CANSparkMax) {
            m_canEncoder.setPosition(0);
        }

        if (m_motorType == MotorType.VictorSPX || m_motorType == MotorType.Spark){
            if (m_encoder != null){
                m_encoder.reset();
            }
        }
    }

    /**
     * Configs the motor's feedforward gains.
     * This must be done before Velocity control mode can be used.
     * @param kS
     * @param kV
     * @param kA
     */
    public void configFeedforward(double kS, double kV, double kA){
        m_feedforward = new SimpleMotorFeedforward(kS, kV, kA);
        m_isFeedforwardConfigured = true;
    }

    /**
     * Does nothing if velocity control is used. (brakes are always on)
     * @param state If true then the motor will apply brakes when percent out is set to 0.
     */
    public void enableBrakeMode(boolean state){
        switch(m_motorType){
            case TalonSRX:
                m_talonSRX.setNeutralMode((state) ? NeutralMode.Brake : NeutralMode.Coast);
                break;
            
            case TalonFX:
                m_talonFX.setNeutralMode((state) ? NeutralMode.Brake : NeutralMode.Coast);
                break;

            case VictorSPX:
                m_victorSPX.setNeutralMode((state) ? NeutralMode.Brake : NeutralMode.Coast);
                break;
            
            case CANSparkMax:
                m_canSparkMax.setIdleMode((state) ? IdleMode.kBrake : IdleMode.kCoast);
                break;
            
            case Spark:
                System.out.println("This motor type does not support brake mode configureation. Motor type: Spark");
                break;
        }
    }
}


