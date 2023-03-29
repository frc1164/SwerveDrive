// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Gripper;


public class Clasp extends CommandBase {
  private final Gripper m_subsystem;
  private final CommandXboxController m_controller;

  /** Creates a new Clasp. */
  public Clasp(Gripper subsystem, CommandXboxController m_controller2) {
    m_subsystem = subsystem;
    m_controller = m_controller2;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_controller.getRawAxis(5)) > 0.2){
      m_subsystem.setClasp(m_controller.getRawAxis(5)/4);
    }
    else {
      m_subsystem.setClasp(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
