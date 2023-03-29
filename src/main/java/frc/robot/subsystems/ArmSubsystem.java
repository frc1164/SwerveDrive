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
import frc.robot.GamePiece;
import frc.robot.Setpoint;
import frc.robot.Constants.ArmConstants;
import frc.robot.GamePiece.GamePieceType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.sensors.*;

public class ArmSubsystem extends SubsystemBase {
  private static CANSparkMax armShoulderMotor;
  private static CANSparkMax armExtensionMotor;
  private static DigitalInput extensionExtendedLimitSwitch;
  private static DigitalInput extensionRetractedLimitSwitch;
  private static DigitalInput armShoulderUpperLimitSwitch;
  private static DigitalInput armShoulderLowerLimitSwitch;
  private static RelativeEncoder TelescopeEncoder;
  private static CANCoder ShoulderEncoder;
  private PIDController thetaPID, radiusPID, setpointThetaPid, setpointRadiusPid;
  private static double tOld, tNew;
  private static double rOld, rNew, thetaOld, thetaNew, radiusOutput, thetaOutput, rError, thetaError, theta, r, setpointX, setpointY;
  private static boolean armSetpoint;
  private static boolean limitSwitchTrigered = false;
  private static double sysStartTime = System.nanoTime() / Math.pow(10, 9);

  //Setpoint variables
  private static Setpoint m_setPoint;

  /** Creates a new ArmShoulder. */
  public ArmSubsystem() {
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
    setpointThetaPid = new PIDController(2, 0, 0);
    setpointRadiusPid = new PIDController(10, 0, 0);
    radiusOutput = 0;
    thetaOutput = 0;
    //Set m_setPoint to null so joystick works
    m_setPoint = null;
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
    if(m_setPoint != null) {
      setArmSetpoint(setpointX, setpointY);
    }
  }

    // This extracts Cone or Cube-specific setpoints from the Setpoint object. Use Liam's setArmSetpoint (setPointX, setPointY) to drive to them.
    public void updateAllArmSetpoints(Setpoint setpoint) {
      m_setPoint = setpoint;
      try{
        if (GamePiece.getGamePiece().equals(GamePieceType.Cone)) {
         setpointX = m_setPoint.X_Cone;
         setpointY = m_setPoint.Y_Cone;
       } else if (GamePiece.getGamePiece().equals(GamePieceType.Cube)) {
         setpointX = m_setPoint.X_Cube;
         setpointY = m_setPoint.Y_Cube;
       }
    } catch (NullPointerException npe){
       System.out.println(npe);
    }
    }

  public boolean setpointTolerance() {
    theta = getShoulderPosition();
    r = getArmLength();
    double x = r * Math.cos(theta);
    double y = r * Math.sin(theta);
    SmartDashboard.putNumber("Tol. x", x - setpointX);
    SmartDashboard.putNumber("Tol. y", y - setpointY);
    if((setpointX + ArmConstants.setpointTolerance >= x) && (setpointX - ArmConstants.setpointTolerance <= x) && (setpointY + ArmConstants.setpointTolerance >= y) && (setpointY - ArmConstants.setpointTolerance <= y)) {
      return true;
    }
    return false;  
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
    // Read in system data
    tNew = System.nanoTime() / Math.pow(10, 9);
    rNew = getArmLength();
    thetaNew = getShoulderPosition();


    // Check if Limit switch has ever been hit - 5 second time limit
    SmartDashboard.putBoolean("Limit Switch Trigger", limitSwitchTrigered);
    SmartDashboard.putNumber("sysStartTime", sysStartTime);
    if (limitSwitchTrigered == false){
      if (getArmExtensionRetractedLimitSwitch()) {
        limitSwitchTrigered = true;
        resetArmExtension();
      }
      else{
        r = 3; // command an extention velocity of 3in/sec
        theta = 0;
      }
    }

    // Calculate Arm Velocity
    double velocityR = -(rOld - rNew)/(tOld - tNew);
    double velocityTheta = -(thetaOld - thetaNew)/(tOld - tNew);
    tOld = tNew;
    rOld = rNew;
    thetaOld = thetaNew;


    // Control arm speed
    rError = r - velocityR;
    thetaError = velocityTheta - theta;
    SmartDashboard.putNumber("Vr - Error", rError);
    SmartDashboard.putNumber("Vtheta - Error", thetaError);
    radiusOutput = radiusOutput + radiusPID.calculate(rError);
    thetaOutput = thetaOutput + thetaPID.calculate(thetaError);
   
    // Motor Limit
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
