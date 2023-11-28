// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Gripper;

public class CubeInit extends CommandBase {
  private final Gripper m_subsystem;
  public static Timer m_timer;

  /** Creates a new CubeInit. */
  public CubeInit(Gripper subsystem) {
    m_subsystem = subsystem;
    m_timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
    m_subsystem.setgripPID(0); //was -16.5
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Should Pos",ArmSubsystem.getShoulderPosition() );
    if (ArmSubsystem.getShoulderPosition() >= (-.9))
      m_subsystem.runGripPID(m_subsystem.gripPosition());
      
  }
   // SmartDashboard.putBoolean("Y_BUTTON", m_controller.getYButton());
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(( m_subsystem.ToFDistance() <= 200) & (Gripper.claspEncoder.getPosition() > -70) || (m_timer.get() >= 3)){
      return true;
    } else {
      return false;
    }
  }
}
