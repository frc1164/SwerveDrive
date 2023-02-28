// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmShoulderCmd extends CommandBase {
  private final ArmSubsystem m_subsystem;
  private final XboxController m_controller;

  /** Creates a new ArmShoulderCommand. */
  public ArmShoulderCmd(ArmSubsystem subsystem, XboxController controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Raw Axis 0", m_controller.getRawAxis(0));
    m_subsystem.setExtensionMotorSpeed(m_controller.getRawAxis(0)/4);
    SmartDashboard.putNumber("Raw Axis 1", m_controller.getRawAxis(1));
    m_subsystem.setRotationMotorSpeed(m_controller.getRawAxis(1)/4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
