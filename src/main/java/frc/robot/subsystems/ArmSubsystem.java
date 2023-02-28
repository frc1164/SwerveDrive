// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
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

  /** Creates a new ArmShoulder. */
  public ArmSubsystem() {
    m_canifier = new CANifier(ArmConstants.armCANifier);
    armShoulderMotor = new CANSparkMax(ArmConstants.armShoulderMotor, MotorType.kBrushless);
    armShoulderMotor.setIdleMode(IdleMode.kBrake);
    armExtensionMotor = new CANSparkMax(ArmConstants.armExtensionMotor, MotorType.kBrushless);
    armExtensionMotor.setIdleMode(IdleMode.kBrake);
    extensionRetractedLimitSwitch = new DigitalInput(0);
    extensionExtendedLimitSwitch = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Upper Limit Switch", getArmShoulderUpperLimitSwitch());
    SmartDashboard.putBoolean("Lower Limit Switch", getArmShoulderLowerLimitSwitch());
    SmartDashboard.putBoolean("Extended Limit Switch", getArmExtensionExtendedLimitSwitch());
    SmartDashboard.putBoolean("Retracted Limit Switch", getArmExtensionRetractedLimitSwitch());
  }

  public void setRotationMotorSpeed(double speed) {
    if(!getArmShoulderUpperLimitSwitch() && (speed > 0)){
      armShoulderMotor.set(0);
    }
    else if(!getArmShoulderLowerLimitSwitch() && (speed < 0)){
      armShoulderMotor.set(0);
    }
    else {
      armShoulderMotor.set(speed);
    }
  }

  public void setExtensionMotorSpeed(double speed) {
    if(getArmExtensionExtendedLimitSwitch() && (speed > 0)){
      armExtensionMotor.set(0);
    }
    else if(getArmExtensionRetractedLimitSwitch() && (speed < 0)){
      armExtensionMotor.set(0);
    }
    else {
      armExtensionMotor.set(speed);
    }
  }

  public boolean getArmShoulderUpperLimitSwitch() {
    return m_canifier.getGeneralInput(GeneralPin.LIMF);
  }

  public boolean getArmShoulderLowerLimitSwitch() {
    return m_canifier.getGeneralInput(GeneralPin.LIMR);
  }

  public boolean getArmExtensionExtendedLimitSwitch() {
    return extensionExtendedLimitSwitch.get();
  }

  public boolean getArmExtensionRetractedLimitSwitch() {
    return extensionRetractedLimitSwitch.get();
  }
}
