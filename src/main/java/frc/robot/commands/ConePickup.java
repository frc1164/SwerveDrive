// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GripperC;
import frc.robot.subsystems.Gripper;

public class ConePickup extends CommandBase {
  private final Gripper m_subsystem;

  /** Creates a new ConePickup. */
  public ConePickup(Gripper subsystem) {
    m_subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setgripPID(-82);
    m_subsystem.Intake(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.runGripPID(m_subsystem.gripPosition());
    m_subsystem.Intake(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.Intake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
