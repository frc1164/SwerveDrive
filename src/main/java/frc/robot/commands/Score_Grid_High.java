// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Gripper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score_Grid_High extends SequentialCommandGroup {
  /** Creates a new Score_Grid_High. 
   * @throws InterruptedException*/
  public Score_Grid_High(ArmSubsystem arm, Gripper gripper) throws InterruptedException {
  ArmSubsystem m_arm = arm;
  Gripper m_gripper = gripper;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(()->m_arm.updateAllArmSetpoints(ArmSetpoints.TEST_SETPOINT_HIGHER)));
    TimeUnit.SECONDS.sleep(2);
    addCommands(new InstantCommand(()->m_gripper.updateAllGripperSetpoints(ArmSetpoints.TEST_SETPOINT_HIGHER)));
    TimeUnit.SECONDS.sleep(2);
    addCommands(new InstantCommand(()->m_arm.updateAllArmSetpoints(ArmSetpoints.TEST_SETPOINT_LOWER)));
  }
}
