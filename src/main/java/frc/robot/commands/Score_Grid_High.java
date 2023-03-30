// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Gripper;

public class Score_Grid_High extends CommandBase {
  private final ArmSubsystem m_arm;
  private final Gripper m_gripper;
  private boolean setpoint1_finished, setpoint2_finished, setpoint3_finished, setpoint4_finished, m_end;
  private Timer delayTimer;

  /** Creates a new Score_Grid_High. */
  public Score_Grid_High(ArmSubsystem arm, Gripper gripper) {
    // Use addRequirements() here to declare subsystem dependencies.
    delayTimer = new Timer();
    m_arm = arm;
    m_gripper = gripper;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delayTimer.reset();
    delayTimer.start();
    setpoint1_finished = false;
    setpoint2_finished = false;
    setpoint3_finished = false;
    setpoint4_finished = false;
    m_end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  // N.B.: Need to figure out how to insert some timing between setpoints.
  @Override
  public void execute() {
    
    if (!setpoint1_finished) {
      //Set the first Setpoint, fall through when first setpoint within tolerance
      m_arm.updateAllArmSetpoints(ArmSetpoints.TOP_NODE);
      m_gripper.updateAllGripperSetpoints(ArmSetpoints.TOP_NODE);
      setpoint1_finished = m_arm.setpointTolerance();
      if (setpoint1_finished) {
        //Sleep some amount between setpoints
        delayTimer.reset();
        // while(delayTimer.get() < 0.5) {}
      }

    } 
    else if (setpoint1_finished && !setpoint2_finished) {
      // Set the second Setpoint, fall through when first two setpoints completed
      m_arm.updateAllArmSetpoints(ArmSetpoints.TOP_NODE_PLACED);
      m_gripper.updateAllGripperSetpoints(ArmSetpoints.TOP_NODE_PLACED);
      setpoint2_finished = m_arm.setpointTolerance();
      if (setpoint2_finished) {
        // Sleep some amount between setpoints
        delayTimer.reset();
        // while(delayTimer.get() < 0.5) {}
      }

    }
    else if (setpoint2_finished && !setpoint3_finished) {
      // Set the second Setpoint, fall through when first two setpoints completed
      m_arm.updateAllArmSetpoints(ArmSetpoints.TOP_NODE_PLACED_AND_SCORED);
      m_gripper.resetandScore();
      setpoint3_finished = m_arm.setpointTolerance();
      if (setpoint3_finished) {
        // Sleep some amount between setpoints
        delayTimer.reset();
        // while(delayTimer.get() < 0.5) {}
      }

    } else if (setpoint3_finished && !setpoint4_finished) {
      //Set the first Setpoint, fall through when first setpoint within tolerance
      m_arm.updateAllArmSetpoints(ArmSetpoints.TOP_NODE);
      m_gripper.updateAllGripperSetpoints(ArmSetpoints.TOP_NODE);
      setpoint4_finished = m_arm.setpointTolerance();
     /*  if (setpoint4_finished) {
        //Sleep some amount between setpoints
        delayTimer.reset();
        // while(delayTimer.get() < 0.5) {}
      } */

    } else if (setpoint1_finished && setpoint2_finished && setpoint3_finished && setpoint3_finished && !m_end) {
    // Set the third Setpoint, fall through when third setpoint completed
      m_arm.updateAllArmSetpoints(ArmSetpoints.STOWED);
      m_gripper.updateAllGripperSetpoints(ArmSetpoints.STOWED);
      m_end = m_arm.setpointTolerance();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_end;
  }
}
