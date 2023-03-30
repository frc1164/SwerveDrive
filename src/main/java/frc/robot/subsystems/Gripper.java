// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.GamePiece;
import frc.robot.Setpoint;
import frc.robot.Constants.GripperC;
import frc.robot.GamePiece.GamePieceType;
import frc.robot.Setpoint.ClaspState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.GeneralPin;
import com.playingwithfusion.TimeOfFlight;


public class Gripper extends SubsystemBase {
  private static CANSparkMax rightDrive;
  private static CANSparkMax leftDrive;
  private static CANSparkMax clasp;

  private static RelativeEncoder claspEncoder;

  public PIDController gripPID;

  private static TimeOfFlight ToF;
  private Setpoint m_setPoint;
  private ClaspState m_Claspstate;

  private static CANifier m_canifier;




  /** Creates a new Gripper. */
  public Gripper() {
    rightDrive = new CANSparkMax(GripperC.rightMotor, MotorType.kBrushless);
    rightDrive.setInverted(GripperC.rightMotorReversed);
    leftDrive = new CANSparkMax(GripperC.leftMotor, MotorType.kBrushless);
    leftDrive.setInverted(GripperC.leftMotorReversed);
    clasp = new CANSparkMax(GripperC.GripperMotor, MotorType.kBrushless);
    ToF = new TimeOfFlight(GripperC.TimeOfFlightSensor);

    m_canifier = new CANifier(GripperC.GripperCANifier);

    claspEncoder = clasp.getEncoder();

    gripPID = new PIDController(0.08, 0.008, 0);
  }
  
  public double gripPosition() {
    return claspEncoder.getPosition();
  }
  
  public void Intake(double speed) {
      rightDrive.set(speed);
      leftDrive.set(speed);
    }
  
  
  
  public void setClasp(double speed) {
    if (speed > 0) {
        if (!getGripperOPENLimitSwitch()) {
            // We are going up and top limit is tripped so stop
            clasp.set(0);
        } else {
            // We are going up but top limit is not tripped so go at commanded speed
            clasp.set(speed);
        }
    } else {
        if (!getGripperCLSDLimitSwitch()) {
            // We are going down and bottom limit is tripped so stop
            clasp.set(0);
        } else {
            // We are going down but bottom limit is not tripped so go at commanded speed
            clasp.set(speed);  
        }
      }
  }
    
  public static void initreverseLeft(){
    leftDrive.setInverted(true);
  }


  public void setgripEncoder() {
    if (!getGripperOPENLimitSwitch())
      claspEncoder.setPosition(0);
  }
  
  public static boolean getGripperCLSDLimitSwitch() {
  // Note: This is incomplete. It may make more sense to just call an input or eject method depending GamePieceType... Talk to Eric
  public void updateAllGripperSetpoints(Setpoint setpoint) {
    m_setPoint = setpoint;
    try{
    if (GamePiece.getGamePiece().equals(GamePieceType.Cone)) {
      //Get the gripper state
      m_Claspstate = m_setPoint.claspCone;
    } 
    else if (GamePiece.getGamePiece().equals(GamePieceType.Cube)) {
      m_Claspstate = m_setPoint.claspCube;
    }
    } catch (NullPointerException npe){
     System.out.println(npe);
    }
    
    //Note: These numbers are almost certainly wrong
    switch (m_Claspstate) {
      case PRELOAD:  this.setgripPID(-82);
               break;
      case SET:  this.setgripPID(-16.5);
               break;
      case OPEN:  this.setgripPID(-1);
               break;
      default: break;
    }
  }

public static boolean getGripperCLSDLimitSwitch() {
    return m_canifier.getGeneralInput(GeneralPin.LIMF);
  }

  public static boolean getGripperOPENLimitSwitch() {
    return m_canifier.getGeneralInput(GeneralPin.LIMR);
  }

  public void setgripPID(double setpoint) {
    gripPID.setSetpoint(setpoint);
  }

  public void runGripPID(double gripPosition) {
    setClasp(gripPID.calculate(gripPosition));
  }

  public double ToFDistance() {
    return ToF.getRange();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gripper Encoder", claspEncoder.getPosition());
    SmartDashboard.putBoolean("CLSD Limit Switch", getGripperCLSDLimitSwitch());
    SmartDashboard.putBoolean("OPEN Limit Switch", getGripperOPENLimitSwitch());
    SmartDashboard.putNumber("ToF Distance", ToF.getRange());
    setgripEncoder();
    //gripper range is (85.0272 units to other end as is -need to check polarity-)

    //Drive to a Clasp setpoint for Autonomous. Note: We should probably test for Autonomous mode here to keep this from messing with Teleop behavior.
    if(m_setPoint != null) {
      this.runGripPID(this.gripPosition());
    }
  }
}
