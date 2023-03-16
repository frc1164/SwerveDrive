// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCmd extends CommandBase {
  private final ArmSubsystem m_subsystem;
  private final XboxController m_controller;
  private double radiusJoystickReading, thetaJoystickReading;
  private double theta, vTheta, r, vR, x, y;

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
    if(Math.abs(m_controller.getRawAxis(5)) > 0.1){
      radiusJoystickReading = m_controller.getRawAxis(5);
    }
    else {
      radiusJoystickReading = 0;
    }

    if(Math.abs(m_controller.getRawAxis(1)) > 0.1){
      thetaJoystickReading =  m_controller.getRawAxis(1);
    }
    else{
      thetaJoystickReading = 0;
    }

    // m_subsystem.setArmVelocity(thetaJoystickReading, radiusJoystickReading);

    // An attempt at accurate arm extension distance
    if(m_subsystem.getArmExtensionRetractedLimitSwitch()) {
      m_subsystem.resetArmExtension();
    }

    // Get arm position
    theta = m_subsystem.getShoulderPosition();
    r = m_subsystem.getArmLength();
    x = r * Math.cos(theta);
    y = r * Math.sin(theta);
    SmartDashboard.putNumber("Arm Theta", theta);
    SmartDashboard.putNumber("Arm r", r);
    SmartDashboard.putNumber("Arm x", x);
    SmartDashboard.putNumber("Arm y", y);

    double Vy = radiusJoystickReading * 10;
    double Vx = thetaJoystickReading * 10;
    SmartDashboard.putNumber("radiusJoystickReading", radiusJoystickReading);
    SmartDashboard.putNumber("thetaJoystickReading", thetaJoystickReading);

    // Inverse Kinematics
    vTheta = (x * Vy - y * Vx) / (Math.pow(x, 2) + Math.pow(y, 2));
    vR = (x * Vx + y * Vy) / Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));  

    m_subsystem.setArmVelocity(vTheta, vR);
    //m_subsystem.setArmVelocity(thetaJoystickReading, radiusJoystickReading * 10);
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
