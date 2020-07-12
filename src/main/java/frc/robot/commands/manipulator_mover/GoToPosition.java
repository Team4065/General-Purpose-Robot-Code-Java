/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.manipulator_mover;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.manipulator_mover.ManipulatorMover;
import frc.robot.RobotMap;
import frc.robot.ExtraMath.*;

public class GoToPosition extends CommandBase {
  /**
   * Creates a new GoToPosition.
   */
  private ManipulatorMover manipulatorMover;
  private Vector3 target;
  private double speed;

  /**
   * Moves the target position at a set rate
   * @param _manipulatorMover the manipulator mover to be controlled
   * @param _target the desired endpoint
   * @param _speed the speed in units per second
   */
  public GoToPosition(ManipulatorMover _manipulatorMover, Vector3 _target, double _speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_manipulatorMover);
    manipulatorMover = _manipulatorMover;
    target = _target;
    speed = _speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Vector3 tempTarget = Vector3.moveTowards(manipulatorMover.getMeasuredEndpoint(), target, speed * RobotMap.DELTA_TIME);
    /*
    double timeJump = 1;
    Vector3 tempTarget = Vector3.slerp(manipulatorMover.getMeasuredEndpoint(), target, timeJump);
    while(tempTarget.sub(manipulatorMover.getMeasuredEndpoint()).magnitude() >  speed * RobotMap.DELTA_TIME){
      timeJump *= 0.75;
      tempTarget = Vector3.slerp(manipulatorMover.getMeasuredEndpoint(), target, timeJump);
    }
    */
    manipulatorMover.setTarget(tempTarget);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return manipulatorMover.getMeasuredEndpoint().sub(target).magnitude() < RobotMap.ACCURACY_TOLERANCE;
  }
}
