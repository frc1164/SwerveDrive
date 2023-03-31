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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.GamePiece;
import frc.robot.GamePiece.GamePieceType;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.ArmCmd;
import frc.robot.commands.BalanceCmd;
import frc.robot.commands.Clasp;
import frc.robot.commands.ConePickup;
import frc.robot.commands.CubePickup;
import frc.robot.commands.Erics_sequential_pickup;
import frc.robot.commands.Liams_sequential_score_grid_high;
import frc.robot.commands.ScoreGridTop;
import frc.robot.commands.Score_Grid_High;
import frc.robot.commands.intake;
import frc.robot.commands.output;
import frc.robot.Constants.GripperC;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Gripper;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import java.util.ArrayList;
import java.util.HashMap;

public class RobotContainer {

        private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        private final ArmSubsystem armSubsystem = new ArmSubsystem();

        private final Gripper m_gripper;
        private final CommandXboxController m_controller;

        SendableChooser<Command> m_chooser = new SendableChooser<>();

        public final HashMap<String, Command> eventMap = new HashMap<>();

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

                GamePiece.setGamePiece(GamePieceType.Cone);

                // Autonomous stuff
                // Read in Autonomous trajectories as multiple paths
                List<PathPlannerTrajectory> trajectory5 = PathPlanner.loadPathGroup("Middle Long", 2, 1);
                List<PathPlannerTrajectory> trajectory6 = PathPlanner.loadPathGroup("Middle Short", 2, 1);
                List<PathPlannerTrajectory> trajectory7 = PathPlanner.loadPathGroup("Right Path", 2, 1);
                List<PathPlannerTrajectory> trajectory8 = PathPlanner.loadPathGroup("Left Path", 2, 1);

                // Populate the Event Map (PathPlanner labels paired with commands)
                eventMap.put("balance", new BalanceCmd(swerveSubsystem));

                // Construct the PathPlanner AutoBuilder (only needs to happen once)
                SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                swerveSubsystem::getPose, // Pose2d supplier
                swerveSubsystem::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                DriveConstants.kDriveKinematics, // SwerveDriveKinematics
                new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
                new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
                swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap, true,// Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
                );

                //Populate the Sendable Chooser with calls to autoBuilder with specific tractories
                m_chooser.addOption("Middle Long", autoBuilder.fullAuto(trajectory5));
                m_chooser.addOption("Middle Short", autoBuilder.fullAuto(trajectory6));
                m_chooser.addOption("Right Path", autoBuilder.fullAuto(trajectory7));
                m_chooser.addOption("Left Path", autoBuilder.fullAuto(trajectory8));
                m_chooser.addOption("None", null);
                Shuffleboard.getTab("Auto").add(m_chooser);

                configureButtonBindings();
        }

        private void configureButtonBindings() {

                // Sets buttons
                Trigger aButton = m_controller.a();
                Trigger bButton = m_controller.b();
                Trigger yButton = m_controller.y();
                Trigger xButton = m_controller.x();
                Trigger lBumper = m_controller.leftBumper();
                Trigger rBumper = m_controller.rightBumper();
                Trigger lDPad = m_controller.povLeft();
                Trigger rDPad = m_controller.povRight();
                Trigger uDPad = m_controller.povUp();
                Trigger dDPad = m_controller.povDown();

                // keybinds
                lBumper.whileTrue(new CubePickup(m_gripper));
                rBumper.whileTrue(new ConePickup(m_gripper));
                xButton.whileTrue(new intake(m_gripper));
                aButton.whileTrue(new output(m_gripper));
                
                //Temporarily bind autonomous commands. For testing commands only!
                lDPad.onTrue(new Score_Grid_High(armSubsystem, m_gripper));                     // Old old - working(ish)
                dDPad.onTrue(new Erics_sequential_pickup(armSubsystem, m_gripper));             // Old (Tursday) - forgot status
                rDPad.onTrue(new Liams_sequential_score_grid_high(armSubsystem, m_gripper));    // New, not tested

                /*
                 * new JoystickButton(driverJoytick, 2).whenPressed(() ->
                 * swerveSubsystem.zeroHeading());
                 */
                new JoystickButton(driverJoytick, 2).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
                new JoystickButton(driverJoytick, 3).onTrue(new BalanceCmd(swerveSubsystem));
        }

        public Command getAutonomousCommand() {
                 return m_chooser.getSelected();
        }
}