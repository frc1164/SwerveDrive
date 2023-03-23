// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCmd extends CommandBase {
  private final ArmSubsystem m_subsystem;
  private final XboxController m_controller;
  private double radiusJoystickReading, thetaJoystickReading;
  private double theta, vTheta, r, rMax, thetaMax, vRadius, x, y;

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
    if(Math.abs(m_controller.getRawAxis(2)) > 0.1){
      radiusJoystickReading = m_controller.getRawAxis(2);
    }
    else if(Math.abs(m_controller.getRawAxis(3)) > 0.1){
      radiusJoystickReading = -m_controller.getRawAxis(3);
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
    radiusJoystickReading *= 10;
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

    // Soft Limits - Retracted
    if(radiusJoystickReading > 10 * (r - ArmConstants.armRetractedSoftStop)) {
      vRadius = (r - ArmConstants.armR0 + 1.5) * -10;
    } 
    // Soft Limits - Extended
    else if(radiusJoystickReading > 10 * (r - ArmConstants.armExtendedSoftStop)) {
      vRadius = (r - ArmConstants.armExtendedSoftStop) * -10;
    } 
    else {
      vRadius = radiusJoystickReading;
    }

    // Soft Limits - Floor
    if(r*Math.cos(theta) > ArmConstants.pivotPointXDistanceFromFloor) {
      rMax = ArmConstants.pivotPointXDistanceFromFloor/Math.sin(theta);
    } 
    // Soft Limits - Bumper
    else if(r*Math.cos(theta) < ArmConstants.pivotPointXDistanceFromBumper) {
      rMax = ArmConstants.pivotPointXDistanceFromBumper/Math.sin(theta);
    } 
    else {
      rMax = -(ArmConstants.pivotPointXDistanceFromBumper*ArmConstants.pivotPointYDistanceFromFloor - ArmConstants.pivotPointXDistanceFromFloor*ArmConstants.pivotPointYDistanceFromBumper) / (ArmConstants.pivotPointYDistanceFromBumper*Math.cos(theta) - ArmConstants.pivotPointYDistanceFromFloor*Math.cos(theta) - ArmConstants.pivotPointXDistanceFromBumper*Math.sin(theta) + ArmConstants.pivotPointXDistanceFromFloor*Math.sin(theta));

    }

    // Soft Limit - Top
    if(thetaJoystickReading > 10 * ArmConstants.TopShoulderSoftStop) {
      vTheta = (theta - ArmConstants.TopShoulderSoftStop) * -5;
    } 
    // Soft Limit - Bumper/Floor
    if(r > rMax) {
      vRadius = (r - rMax) * -5;
    }

    m_subsystem.setArmVelocity(vTheta, vRadius);
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
