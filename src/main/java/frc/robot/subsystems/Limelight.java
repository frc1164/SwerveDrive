// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight"); 

  public Limelight() {}
public double getTX() {
    return table.getEntry("tx").getDouble(0.0);
}

public double getTY() {
    return table.getEntry("ty").getDouble(0.0);
}

public double getTA() {
    return table.getEntry("ta").getDouble(0.0);
}

public void setLLPipeline(int pipelineNumber){
  Number numObj = (Number)pipelineNumber;
  table.getEntry("pipeline").setNumber(numObj);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("X offset", getTX());
    SmartDashboard.putNumber("Y offset", getTY());
    SmartDashboard.putNumber("A offset", getTA());
  }

}
