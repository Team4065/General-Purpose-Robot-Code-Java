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
import frc.robot.Utility.RamseteCommandBuilder2;
import frc.robot.commands.Drivetrain.ArcadeDrive2;
import frc.robot.commands.Drivetrain.ArcadeDrivePercent;
import frc.robot.commands.Drivetrain.FindTrackWidth;
import frc.robot.subsystems.DifferentialDrivetrain2;
import frc.robot.Utility.Motors.*;






/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  DifferentialDrivetrain2 m_drivetrain = new DifferentialDrivetrain2(
  new TalonFX(2),
  new TalonFX(1),
  new Motor[]{new TalonFX(3)},
  new Motor[]{new TalonFX(16)},
  0.1016,
  1.0 / 9.09,
  0.7316606976086597,
  true,
  true
  );
  //gear ratio: 

  public Joystick m_controller = new Joystick(0);

  public RamseteCommandBuilder2 path = new RamseteCommandBuilder2(m_drivetrain, "Forward");
  

  public RobotContainer() {
    m_drivetrain.configLeftFeedForward(-0.815858358638, -1.0991544081);
    m_drivetrain.configRightFeedForward(-0.813737121479, -1.09103641734);
    m_drivetrain.enableBrakes();
    //m_drivetrain.configLeftPID(0.0, -0.05, 0);
    //m_drivetrain.configRightPID(0.0, -0.05, 0);

    //m_drivetrain.setDefaultCommand(new ArcadeDrivePercent(m_drivetrain, m_controller));
    m_drivetrain.setDefaultCommand(new ArcadeDrive2(m_drivetrain, m_controller, 0.2, 10));
    
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
    //return new FindTrackWidth(m_drivetrain);
    return path.getCommand();
    //return m_drivetrain.findLeftFeedForwardGains;
    //return m_drivetrain.findRightFeedForwardGains;
    //return new ExampleCommand();
  }
}
