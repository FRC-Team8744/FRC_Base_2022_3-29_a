// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberUp extends CommandBase {
  private final Climber m_climber;
  private final double m_SpeedLimit;

  /** Constructor for MoveArmUp.
   *  @param thisArmLift  The subsystem this command will act on
   *  @param liftSpeed The rate the arm will move (should be in range 0..1 )
   */
  public ClimberUp(Climber thisClimber, double liftSpeed) {
    m_climber = thisClimber;
    m_SpeedLimit = liftSpeed;

    addRequirements(thisClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.raiseArm(m_SpeedLimit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
