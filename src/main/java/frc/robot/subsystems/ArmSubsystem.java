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
import edu.wpi.first.math.controller.PIDController;
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
  private PIDController thetaPID, radiusPID;
  private static double tOld, tNew;
  private static double rOld, rNew, thetaOld, thetaNew, radiusOutput, thetaOutput, rError, thetaError;

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
    thetaPID = new PIDController(ArmConstants.thetaP, ArmConstants.thetaI, ArmConstants.thetaD);
    //thetaPID = new PIDController(ArmConstants.thetaP, ArmConstants.thetaI, ArmConstants.thetaD);
    radiusPID = new PIDController(ArmConstants.radiusP, ArmConstants.radiusI, ArmConstants.radiusD);
    //radiusPID = new PIDController(ArmConstants.radiusP, ArmConstants.radiusI, ArmConstants.radiusD);
    radiusOutput = 0;
    thetaOutput = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putBoolean("Upper Limit Switch", getArmShoulderUpperLimitSwitch());
    // SmartDashboard.putBoolean("Lower Limit Switch", getArmShoulderLowerLimitSwitch());
    // SmartDashboard.putBoolean("Extended Limit Switch", getArmExtensionExtendedLimitSwitch());
    // SmartDashboard.putBoolean("Retracted Limit Switch", getArmExtensionRetractedLimitSwitch());
    // SmartDashboard.putNumber("Telescope Position", getTelescopePosition());
    // SmartDashboard.putNumber("Shoulder Position", getShoulderPosition());
  }

  public void setRotationMotorSpeed(double speed) {

    if(/*!getArmShoulderUpperLimitSwitch() &&*/ (speed < 0) &&(getShoulderPosition() >= ArmConstants.TopShoulderLimit)){
      armShoulderMotor.set(0);
      thetaPID.reset();
    }
    else if(/*!getArmShoulderLowerLimitSwitch() &&*/ (speed > 0) && (getShoulderPosition() <= ArmConstants.BottomShoulderLimit)){
      armShoulderMotor.set(0);
      thetaPID.reset();
    }
    else {
      armShoulderMotor.set(speed);
    }
  }

  public void setExtensionMotorSpeed(double speed) {
     if(getArmExtensionExtendedLimitSwitch() && (speed > 0)/* && (getShoulderPosition() >= ArmConstants.TopTelescopeLimit) */){
       armExtensionMotor.set(0);
       //radiusPID.reset();
     }
     else if(getArmExtensionRetractedLimitSwitch() && (speed < 0)/* && (getShoulderPosition() <= ArmConstants.BottomTelescopeLimit) */){
       armExtensionMotor.set(0);
       //radiusPID.reset();
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

  public void setArmVelocity(double theta, double r) {
    // Read in Sensors
    thetaJoystickReading = theta;
    radiusJoystickReading = r;

    theta = m_subsystem.getShoulderPosition();
    r = m_subsystem.getArmLength();
    x = r * Math.cos(theta);
    y = r * Math.sin(theta);
    SmartDashboard.putNumber("Arm Theta", theta);
    SmartDashboard.putNumber("Arm r", r);
    SmartDashboard.putNumber("Arm x", x);
    SmartDashboard.putNumber("Arm y", y);



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
    
    // Soft Limits - Floor/Bumper - Find Limit Position
    if (r < (ArmConstants.yBumper / Math.sin(ArmConstants.thetaBumper))) {
      thetaLimit = Math.asin(ArmConstants.yBumper / r); // Bumper Limit
    }
    else if (r > (ArmConstants.yFloor / Math.sin(ArmConstants.thetaBumper))) {
      thetaLimit = Math.asin(ArmConstants.yFloor / r); //Floor Limit
    }
    else {
      thetaLimit = ArmConstants.thetaBumper; // In-between Limit
    }
    SmartDashboard.putNumber("Theta Limit", thetaLimit);

    // Soft Limits - Floor/Bumper - Set Speed
    if(thetaJoystickReading > 5 * (theta - thetaLimit)) {   
      vTheta = (theta - thetaLimit) * 5;  // floor/bumper limit speed
    } 

    // Soft Limit - Top
    if(thetaJoystickReading < 5 * (theta - ArmConstants.TopShoulderSoftStop)) {
      vTheta = (theta - ArmConstants.TopShoulderSoftStop) * 5;
    } 

    
    // Reset variable names to work with below code (yes we should have just made names consistant, but i'm out of time)
    r = vRadius;
    theta = vTheta;


    // Read in system data
    tNew = System.nanoTime() / Math.pow(10, 9);
    rNew = getArmLength();
    thetaNew = getShoulderPosition();

    double velocityR = -(rOld - rNew)/(tOld - tNew);
    double velocityTheta = -(thetaOld - thetaNew)/(tOld - tNew);

    tOld = tNew;
    rOld = rNew;
    thetaOld = thetaNew;
    // theta = 0;
    // r = 0;
    rError = r - velocityR;
    thetaError = velocityTheta - theta;
    SmartDashboard.putNumber("Vr - Error", rError);
    SmartDashboard.putNumber("Vtheta - Error", thetaError);
    radiusOutput = radiusOutput + radiusPID.calculate(rError);
    thetaOutput = thetaOutput + thetaPID.calculate(thetaError);
    if(Math.abs(radiusOutput) > 1) radiusOutput = Math.signum(radiusOutput);
    if(Math.abs(thetaOutput) > 1) thetaOutput = Math.signum(thetaOutput);
    // if(Math.abs(radiusOutput) > ArmConstants.radiusOutputMax) {
		//   radiusOutput = Math.signum(radiusOutput) * ArmConstants.radiusOutputMax;
    // }
    // if(Math.abs(thetaOutput) > ArmConstants.thetaOutputMax) {
		//   thetaOutput = Math.signum(radiusOutput) * ArmConstants.thetaOutputMax;
    // }
    setExtensionMotorSpeed(radiusOutput);
    SmartDashboard.putNumber("Theta output", thetaOutput);
    setRotationMotorSpeed(thetaOutput);
  }

  public void armControl(double theta, double r) {
    if(m_setPoint != null && (Math.abs(theta) > 0 || Math.abs(r) > 0)){
      armSetpoint = false;
      m_setPoint = null;
    }
    setArmVelocity(theta, r);
  }

  public void setArmSetpoint(double x, double y) {
    double setpointTheta, setpointR, vTheta, vThetaPid, vR, vRPid, thetaLimit;
    setpointX = x;
    setpointY = y;
    setpointRadiusPid.reset();
    setpointThetaPid.reset();
    if(!armSetpoint){
      armSetpoint = true;
    }

    setpointTheta = Math.atan(y / x);
    setpointR = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

    vThetaPid = setpointThetaPid.calculate((setpointTheta - getShoulderPosition()));
    vRPid = setpointRadiusPid.calculate(setpointR - getArmLength());

    vTheta = vThetaPid;
    vR = vRPid;
    // Soft Limits - Retracted
    if(vRPid > 5 * (r - ArmConstants.armRetractedSoftStop)) {
      vR = (r - ArmConstants.armRetractedSoftStop) * 5;
    } 
    // Soft Limits - Extended
    else if(vRPid < 5 * (r - ArmConstants.armExtendedSoftStop)) {
      vR = (r - ArmConstants.armExtendedSoftStop) * 5;
    } 
    else {
      vR = vRPid;
    }
    
    // Soft Limits - Floor/Bumper - Find Limit Position
    if (r < (ArmConstants.yBumper / Math.sin(ArmConstants.thetaBumper))) {
      thetaLimit = Math.asin(ArmConstants.yBumper / r); // Bumper Limit
    }
    else if (r > (ArmConstants.yFloor / Math.sin(ArmConstants.thetaBumper))) {
      thetaLimit = Math.asin(ArmConstants.yFloor / r); //Floor Limit
    }
    else {
      thetaLimit = ArmConstants.thetaBumper; // In-between Limit
    }
    SmartDashboard.putNumber("Theta Limit", thetaLimit);

    // Soft Limits - Floor/Bumper - Set Speed
    if(vThetaPid > 5 * (theta - thetaLimit)) {   
      vTheta = (theta - thetaLimit) * 5;  // floor/bumper limit speed
    } 

    // Soft Limit - Top
    if(vThetaPid < 5 * (theta - ArmConstants.TopShoulderSoftStop)) {
      vTheta = (theta - ArmConstants.TopShoulderSoftStop) * 5;
    } 


    if(Math.abs(vTheta) > 1) {
      vTheta = Math.signum(vTheta);
    }
    if(Math.abs(vR) > 15) {
      vR = Math.signum(vR)*15;
    }
    setArmVelocity(vTheta, vR);
  }
}
