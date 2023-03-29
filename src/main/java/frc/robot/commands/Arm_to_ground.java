// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Gripper;

public class Arm_to_ground extends CommandBase {
  private final ArmSubsystem m_arm;
  private final Gripper m_gripper;
  private boolean m_end;
  
  /** Creates a new Arm_to_ground. */
  public Arm_to_ground(ArmSubsystem arm, Gripper gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_gripper = gripper;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_end) {
      // Set the Setpoint, fall through when setpoint is completed
        m_arm.updateAllArmSetpoints(ArmSetpoints.FLOOR_INTAKING);
        m_gripper.updateAllGripperSetpoints(ArmSetpoints.FLOOR_INTAKING);
        m_end = m_arm.setpointTolerance();
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_end;
  }
}
