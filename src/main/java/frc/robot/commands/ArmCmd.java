// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.math.controller.PIDController;
//imporMath; // Consider this library suspect

public class ArmCmd extends CommandBase {
  private final ArmSubsystem m_subsystem;
  private final XboxController m_controller;

  // Initilize PID
  private PIDController ShoulderPID;
  private PIDController ExtendPID;

  // Shoulder PID Gains
  private final double ShoulderP = 0;
  private final double ShoulderI = 0;
  private final double ShoulderD = 0;

  // Extention PID Gains
  private final double ExtendP = 0;
  private final double ExtendI = 0;
  private final double ExtendD = 0;

  // Initilize Position Variables
  private final double VthetaMotor = 0;
  private final double VrMotor = 0;
  private final double X = 0; 
  private final double Y = 0; 

  // Arm Control Perameters
  private final double VMax = 1; // (in/s) max speed that the arm can move (1 norm)
  private final double K_limit = 10;// Arm limit return gain


  // Arm Position Limits
  private final double thetaMax = 1; // (rad) arm max angle in the theta direction
  private final double rMax = 47; // (in) Max arm extention
  private final double rMin = 26; // (in) Min arm extention
  private final double YFloor = -36;// (in) Floor level
  private final double YBumper = -28;// (in) Bumper level
  private final double X1 = 36;// (in) Front of bumper - sorta
  private final double X2 = 30;// (in) Back of bumper - sorta


  /** Creates a new ArmShoulderCommand. */
  public ArmCmd(ArmSubsystem subsystem, XboxController controller) {
    m_subsystem = subsystem;
    m_controller = controller;
    addRequirements(subsystem);
    this.ShoulderPID = new PIDController(ShoulderP, ShoulderI, ShoulderD);
    this.ExtendPID = new PIDController(ExtendP, ExtendI, ExtendD);
  }

  // Called when the command is initially scheduled.

  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate current X and Y position
    X = r * cos(theta);
    Y = r * sin(theta);


    double Vx;
    double Vy;

    // Controller Deadband Code - X axis
    if(m_controller.getRawAxis(2) > 0.02){
      Vx = m_controller.getRawAxis(2);
    }
    else if(m_controller.getRawAxis(3) > 0.02){
      Vx = m_controller.getRawAxis(3);
    }
    else {
      Vx = m_controller.getRawAxis(0);
    }


    // Controller Deadband Code - Y axis
    if(m_controller.getRawAxis(1) > 0.02){
      Vy = m_controller.getRawAxis(1);
    }
    else if(m_controller.getRawAxis(1) > 0.02){
      Vy = m_controller.getRawAxis(1);
    }
    else {
      Vy = m_controller.getRawAxis(1);
    }


  
    // Position Limit
      // Top Limit
      double Ymax = X * java.lang.Math.sin(thetaMax);

      if ((Ymax < Y) && (Vx < (Ymax - Y) * K_limit)){
        Vy = (Ymax - Y) * K_limit;
      }

      // Bottom Limit
      // Limit Equation
      double Ymin;
      if (X > X1)
        Ymin = YFloor;
      else if (X < X2) 
        Ymin = YBumper;
      else
        Ymin = ((YBumper - YFloor) / (X1 - X2)) * X + YFloor;
      
  
      // Limit
      if ((Ymin > Y) && (Yx < (Ymin - Y) * K_limit)){
        Vy = (Ymin - Y) * K_limit;
      }

      // Front Limit
      double Xmax = Math.sqrt(Math.pow(rMax, 2) + Math.pow(Y, 2));

      if ((Xmax < X) && (Vx > (Xmin - X) * K_limit)){
        Vx = (Xmax - X) * K_limit;
      }

      // Back Limit
      double Xmin = Math.sqrt(Math.pow(rMin, 2) + Math.pow(Y, 2));

      if ((Xmin > X) && (Vx < (Xmin - X) * K_limit)){
        Vx = (Xmax - X) * K_limit;
      }

    // Velocity Limits - X
    if (Math.abs(Vx) < VMax) {
      Vx = VxMax * Math.signum(Vx);
    }

    // Velocity Limits - Y
    if (Math.abs(Vy) < VMax) {
      Vy = VyMax * Math.signum(Vy);
    }


    // Inverse Kinematics
    double Vtheta =  (sec(theta))^2 * (Vy*X - Vx * y) / X^2;
    double Vr = (X * Vx + Y * Vy) / Math.sqrt(Math.pow(X, 2) + Math.pow(Y, 2)); 

    // Velocity PID Controler
    VthetaMotor = VthetaMotor + ShoulderPID.calculate(theta);
    VrMotor = VrMotor + ExtendPID.calculate(r);

    // Command Motors
    m_subsystem.setRotationMotorSpeed(VthetaMotor);
    m_subsystem.setExtensionMotorSpeed(VrMotor);

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
