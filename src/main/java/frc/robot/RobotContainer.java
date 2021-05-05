/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utility.Gyro;
import frc.robot.Utility.Motor;
import frc.robot.Utility.PathLoader;
import frc.robot.Utility.RamseteCommandBuilder;
import frc.robot.Utility.RamseteCommandBuilder2;
import frc.robot.Utility.Motor.MotorType;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Drivetrain.ArcadeDrive2;
import frc.robot.commands.Drivetrain.TankDrive;
import frc.robot.subsystems.DifferentialDrivetrain;
import frc.robot.subsystems.DifferentialDrivetrain2;
import frc.robot.Utility.Motors.*;






/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  
  Encoder m_leftEncoder = new Encoder(4, 5);
  Encoder m_rightEncoder = new Encoder(6, 7);

  DifferentialDrivetrain2 m_drivetrain = new DifferentialDrivetrain2(
    new Spark(0, m_leftEncoder), 
    new Spark(1, m_rightEncoder), 
    new Spark[]{}, 
    new Spark[]{},
    0.070, 1./120., 0.141, false, false);

  public Joystick m_controller = new Joystick(0);

  public RamseteCommandBuilder2 path = new RamseteCommandBuilder2(m_drivetrain, "Unnamed");
  

  public RobotContainer() {
    
    m_leftEncoder.setDistancePerPulse(1./12.);
    m_rightEncoder.setDistancePerPulse(1./12.);

    m_leftEncoder.setSamplesToAverage(100);
    m_rightEncoder.setSamplesToAverage(100);

    m_drivetrain.configLeftFeedForward(0.7850566808843115, 0.01652176759932694);
    m_drivetrain.configRightFeedForward(0.7506262622110075, 0.01654768757649323);
    
    //m_drivetrain.configLeftPID(0.0, 0.005, 0);
    //m_drivetrain.configRightPID(0.0, 0.005, 0);

    m_drivetrain.setDefaultCommand(new ArcadeDrive2(m_drivetrain, m_controller, 1, 180));
    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return path.getCommand();
    //return m_drivetrain.findLeftFeedForwardGains;
    //return m_drivetrain.findRightFeedForwardGains;
    //return new ExampleCommand();
  }
}
