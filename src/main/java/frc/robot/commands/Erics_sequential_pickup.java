// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Erics_sequential_pickup extends SequentialCommandGroup {
  private final ArmSubsystem m_arm;
  private final Gripper m_gripper;

  /** Creates a new Erics_sequential_score_High. */
  public Erics_sequential_pickup(ArmSubsystem arm, Gripper gripper) {
    m_arm = arm;
    m_gripper = gripper;
    addRequirements(arm);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Arm_to_Floor_intake(m_arm, m_gripper),
      new WaitCommand(1),
//      new InstantCommand(sams_commmand, m_gripper),
      new WaitCommand(1),
      new Arm_to_Stowed(m_arm, m_gripper)
      );
  }
}
