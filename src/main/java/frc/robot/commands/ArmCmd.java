// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCmd extends CommandBase {
  private final ArmSubsystem m_subsystem;
  private final XboxController m_controller;

  /** Creates a new ArmShoulderCommand. */
  public ArmCmd(ArmSubsystem subsystem, XboxController controller) {
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
    if(m_controller.getRawAxis(2) > 0.2){
      m_subsystem.setExtensionMotorSpeed(-m_controller.getRawAxis(2)/2);
    }
    else if(m_controller.getRawAxis(3) > 0.2){
      m_subsystem.setExtensionMotorSpeed(m_controller.getRawAxis(3)/2);
    }
    else {
      m_subsystem.setExtensionMotorSpeed(m_controller.getRawAxis(0));
    }

    if(Math.abs(m_controller.getRawAxis(1)) > 0.1){
      m_subsystem.setRotationMotorSpeed(m_controller.getRawAxis(1)/2);
    }
    else{
      m_subsystem.setRotationMotorSpeed(0);
    }
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
