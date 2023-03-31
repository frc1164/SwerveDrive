// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.DriveConstants;
/** Add your docs here. */

public class RobotOdometry {
    private static final RobotOdometry robotOdometry = new RobotOdometry();
    private SwerveDrivePoseEstimator estimator;
    private SwerveModulePosition[] defaultPositions =
    new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
    };
    
    private RobotOdometry() {
      estimator =
          new SwerveDrivePoseEstimator(
              DriveConstants.kDriveKinematics,
              new Rotation2d(),
              defaultPositions,
              new Pose2d());
    }
  
    public static RobotOdometry getInstance() {
      return robotOdometry;
    }
  
    public SwerveDrivePoseEstimator getPoseEstimator() {
      return estimator;
    }
  }
