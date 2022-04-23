// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmLift;
import frc.robot.subsystems.HookArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArmAndHooks extends ParallelCommandGroup {
  /** Creates a new MoveArmAndHooks. */
  public MoveArmAndHooks(ArmLift thisArmLift, HookArm this_arm, double liftSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveArmTime(thisArmLift, liftSpeed, 1000),
      new SetClimbArmAngle(-0.35,0.8, this_arm, 1000)
    );
  }
}
