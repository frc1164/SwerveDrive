// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class AutoIntake extends CommandBase {
  private final Gripper m_subsystem;
  private final Timer m_timer;
  private boolean cmdFinish;
  /** Creates a new AutoIntake. */
  public AutoIntake(Gripper subsystem) {
    m_subsystem = subsystem;
    m_timer = new Timer();
    cmdFinish = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.Intake(0);
    m_timer.reset();
    m_timer.start();
    cmdFinish = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Timer", m_timer.get());
    if(m_subsystem.ToFDistance()<130 || m_timer.get() >= 2) {
      m_subsystem.Intake(0);
      cmdFinish = true;
    }else{
      m_subsystem.Intake(.75);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_subsystem.Intake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmdFinish;
  }
}
