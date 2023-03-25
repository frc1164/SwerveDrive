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
  private double theta, vTheta, r, rMax, thetaMax, vRadius, x, y, thetaLimit;

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
    radiusJoystickReading *= 20;
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
    vTheta = thetaJoystickReading;
    vRadius = radiusJoystickReading;

    // Soft Limits - Retracted
    if(radiusJoystickReading > 5 * (r - ArmConstants.armRetractedSoftStop)) {
      vRadius = (r - ArmConstants.armRetractedSoftStop) * 5;
    } 
    // Soft Limits - Extended
    else if(radiusJoystickReading < 5 * (r - ArmConstants.armExtendedSoftStop)) {
      vRadius = (r - ArmConstants.armExtendedSoftStop) * 5;
    } 
    else {
      vRadius = radiusJoystickReading;
    }
    
    // // Soft Limits - Floor/Bumper
    // if (r < (ArmConstants.yBumper / Math.sin(ArmConstants.thetaBumper))) {
    //   thetaLimit = Math.asin(ArmConstants.yBumper / r); // Bumper Limit
    // }
    // else if (r > (ArmConstants.yFloor / Math.sin(ArmConstants.thetaBumper))) {
    //   thetaLimit = Math.asin(ArmConstants.yFloor / r); //Floor Limit
    // }
    // else {
    //   thetaLimit = ArmConstants.thetaBumper; // In-between Limit
    // }


    // if(thetaJoystickReading > 1 * (theta - thetaLimit)) {   
    //   vTheta = (theta - thetaLimit) * 1;  // floor/bumper limit speed
    // } 


    // Soft Limit - Top
    if(thetaJoystickReading < 4 * (theta - ArmConstants.TopShoulderSoftStop)) {
      vTheta = (theta - ArmConstants.TopShoulderSoftStop) * 4;
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
