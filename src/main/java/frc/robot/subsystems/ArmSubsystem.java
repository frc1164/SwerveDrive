// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.ctre.phoenix.CANifier.PinValues;
import com.ctre.phoenix.sensors.*;

public class ArmSubsystem extends SubsystemBase {
  private static CANifier m_canifier;
  private static CANSparkMax armShoulderMotor;
  private static CANSparkMax armExtensionMotor;
  private static DigitalInput extensionExtendedLimitSwitch;
  private static DigitalInput extensionRetractedLimitSwitch;
  private static DigitalInput armShoulderUpperLimitSwitch;
  private static DigitalInput armShoulderLowerLimitSwitch;
  private static RelativeEncoder TelescopeEncoder;
  private static CANCoder ShoulderEncoder;

  /** Creates a new ArmShoulder. */
  public ArmSubsystem() {
    m_canifier = new CANifier(ArmConstants.armCANifier);
    armShoulderMotor = new CANSparkMax(ArmConstants.armShoulderMotor, MotorType.kBrushless);
    armShoulderMotor.setIdleMode(IdleMode.kBrake);
    ShoulderEncoder = new CANCoder(ArmConstants.CANCoderid);
    armExtensionMotor = new CANSparkMax(ArmConstants.armExtensionMotor, MotorType.kBrushless);
    armExtensionMotor.setIdleMode(IdleMode.kBrake);
    TelescopeEncoder = armExtensionMotor.getEncoder();
    extensionRetractedLimitSwitch = new DigitalInput(0);
    extensionExtendedLimitSwitch = new DigitalInput(1);
    armShoulderLowerLimitSwitch = new DigitalInput(2);
    armShoulderUpperLimitSwitch = new DigitalInput(3);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Upper Limit Switch", getArmShoulderUpperLimitSwitch());
    SmartDashboard.putBoolean("Lower Limit Switch", getArmShoulderLowerLimitSwitch());
    SmartDashboard.putBoolean("Extended Limit Switch", getArmExtensionExtendedLimitSwitch());
    SmartDashboard.putBoolean("Retracted Limit Switch", getArmExtensionRetractedLimitSwitch());
    SmartDashboard.putNumber("Telescope Position", getTelescopePosition());
    SmartDashboard.putNumber("Shoulder Position", getShoulderPosition());
  }

  public void setRotationMotorSpeed(double speed) {

    if(/*!getArmShoulderUpperLimitSwitch() &&*/ (speed < 0) &&(getShoulderPosition() >= ArmConstants.TopShoulderLimit)){
      armShoulderMotor.set(0);
    }
    else if(/*!getArmShoulderLowerLimitSwitch() &&*/ (speed > 0) && (getShoulderPosition() <= ArmConstants.BottomShoulderLimit)){
      armShoulderMotor.set(0);
    }
    else {
      armShoulderMotor.set(speed);
    }
  }

  public void setExtensionMotorSpeed(double speed) {
     if(getArmExtensionExtendedLimitSwitch() && (speed > 0)/* && (getShoulderPosition() >= ArmConstants.TopTelescopeLimit) */){
       armExtensionMotor.set(0);
     }
     else if(getArmExtensionRetractedLimitSwitch() && (speed < 0)/* && (getShoulderPosition() <= ArmConstants.BottomTelescopeLimit) */){
       armExtensionMotor.set(0);
     }
     else {
       armExtensionMotor.set(speed);
     }
    //armExtensionMotor.set(speed);
  }

  public boolean getArmShoulderUpperLimitSwitch() {
    return armShoulderUpperLimitSwitch.get();
  }

  public boolean getArmShoulderLowerLimitSwitch() {
    return armShoulderLowerLimitSwitch.get();
  }

  public boolean getArmExtensionExtendedLimitSwitch() {
    return extensionExtendedLimitSwitch.get();
  }

  public boolean getArmExtensionRetractedLimitSwitch() {
    return extensionRetractedLimitSwitch.get();
  }

  public double getTelescopePosition() {
    return TelescopeEncoder.getPosition();
  }

  public double getShoulderPosition() {
    double angle = ShoulderEncoder.getAbsolutePosition() * Math.PI/180.0;
    angle -= ArmConstants.ShoulderEncoderOffset;
    if (angle > Math.PI) {
      angle -= Math.PI*2;
    }
    return angle * (ArmConstants.ShoulderEncoderRevsersed ? -1.0 : 1.0);
  }

  public double getArmLength() {
    return ArmConstants.armR0 + ArmConstants.extensionPerRotation * getTelescopePosition();
  }

  public void resetArmExtension() {
    TelescopeEncoder.setPosition(0);
  }
}
