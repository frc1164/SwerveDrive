package frc.robot;

import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.ArmCmd;
import frc.robot.commands.BalanceCmd;
import frc.robot.commands.Clasp;
import frc.robot.commands.ConePickup;
import frc.robot.commands.CubePickup;
import frc.robot.commands.intake;
import frc.robot.commands.output;
import frc.robot.Constants.GripperC;
import frc.robot.subsystems.Gripper;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();

    private final Gripper m_gripper;
    private final CommandXboxController m_controller;

    private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
    private final XboxController armController = new XboxController(1);

    public RobotContainer() {

        m_gripper = new Gripper();
        m_controller = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
    
        
        m_gripper.setDefaultCommand(new Clasp(m_gripper, m_controller));

        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                () -> driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        armSubsystem.setDefaultCommand(new ArmCmd(
                armSubsystem, 
                armController));

        configureButtonBindings();
    }

    private void configureButtonBindings() {

        //Sets buttons
        Trigger aButton = m_controller.a();
        Trigger bButton = m_controller.b();
        Trigger yButton = m_controller.y();
        Trigger xButton = m_controller.x();
        Trigger lBumper = m_controller.leftBumper();
        Trigger rBumper = m_controller.rightBumper();
        //keybinds
        lBumper.whileTrue(new CubePickup(m_gripper));
        rBumper.whileTrue(new ConePickup(m_gripper));
        xButton.whileTrue(new intake (m_gripper));
        aButton.whileTrue(new output (m_gripper));
        bButton.onTrue(new InstantCommand(() -> Gripper.gripToggle()));


        /* new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading()); */
        new JoystickButton(driverJoytick, 2).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
        new JoystickButton(driverJoytick, 11).onTrue(new BalanceCmd(swerveSubsystem));
}

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                List.of(
                        new Translation2d(1.0, 0.0),
                        new Translation2d(1.0, -1.0)),
                new Pose2d(2.0, -1.0, Rotation2d.fromDegrees(180.0)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0.0, 0.0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0.0, 0.0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0.0, 0.0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                DriveConstants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}