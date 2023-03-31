// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Gripper;
import frc.robot.commands.Setpoints.TOP_NODE;
import frc.robot.commands.Setpoints.TOP_NODE_PLACED;
import frc.robot.commands.Setpoints.TOP_NODE_PLACED_AND_SCORED;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Liams_sequential_score_grid_high extends SequentialCommandGroup {
  private final ArmSubsystem m_arm;
  private final Gripper m_gripper;

  /** Creates a new Liams_sequential_score_grid_high. */
  public Liams_sequential_score_grid_high(ArmSubsystem arm, Gripper gripper) {
    m_arm = arm;
    m_gripper = gripper;
    addRequirements(arm);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new TOP_NODE(m_arm, m_gripper),
        new WaitCommand(1),
        new TOP_NODE_PLACED(m_arm, m_gripper),
        new WaitCommand(1),
        new TOP_NODE_PLACED_AND_SCORED(m_arm, m_gripper),
        new WaitCommand(1),
        new TOP_NODE_PLACED(m_arm, m_gripper),
        new WaitCommand(1),
        new TOP_NODE(m_arm, m_gripper),
        new WaitCommand(1));
  }
}
