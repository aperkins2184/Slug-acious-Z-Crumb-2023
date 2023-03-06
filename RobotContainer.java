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
import frc.robot.commands.ControlerCommand;
import frc.robot.subsystems.SwerveSubSystem;

public class RobotContainer {

    private final SwerveSubSystem swerveSubsystem = new SwerveSubSystem();

    private final Joystick driverJoytick = new Joystick(Constants.kDriverControllerPort);

    public RobotContainer() {
        
        swerveSubsystem.setDefaultCommand(new ControlerCommand(
                swerveSubsystem,
                () -> -driverJoytick.getRawAxis(Constants.kDriverYAxis),
                () -> driverJoytick.getRawAxis(Constants.kDriverXAxis),
                () -> driverJoytick.getRawAxis(Constants.kDriverRotAxis),
                () -> !driverJoytick.getRawButton(Constants.kDriverFieldOrientedButtonIdx)));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
      boolean aPressed = driverJoytick.getRawButtonPressed(2);
      if (aPressed)
      {
        swerveSubsystem.zeroHeading();
      }
       
    }

    public Command getAutonomousCommand() {
        //Create trajectory settings
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                Constants.kMaxSpeedMetersPerSecond,
                Constants.kMaxAccelerationMetersPerSecondSquared)
                        .setKinematics(Constants.kDriveKinematics);

        //Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(Constants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new edu.wpi.first.math.controller.ProfiledPIDController(
                Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        //Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                swerveSubsystem::getPose,
                Constants.kDriveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);

        //Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules()));
    }
}