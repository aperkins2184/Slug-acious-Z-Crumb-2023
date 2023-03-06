// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogGyro;
//mport edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.util.sendable.Sendable;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubSystem extends SubsystemBase {
// defines front left swerve drive module
private final SwerveModule frontLeft = new SwerveModule(
  Constants.kFrontLeftDriveMotorPort,
  Constants.kFontLeftTurningDrivePort,
  Constants.kFrontLeftDriveEncoderReversed,
  Constants.kFrontLeftTurningEncoderRevered,
  Constants.kFrontLeftDriveAbsoluteEncoderPort,
  Constants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
  Constants.kFrontLeftDriveAbsoluteEncoderReversed);
// defines front right swerve drive module
  private final SwerveModule frontRight = new SwerveModule(
  Constants.kFrontRightDriveMotorPort,
  Constants.kFontRightTurningDrivePort,
  Constants.kFrontRightDriveEncoderReversed,
  Constants.kFrontRightTurningEncoderRevered,
  Constants.kFrontRightDriveAbsoluteEncoderPort,
  Constants.kFrontRightDriveAbsoluteEncoderOffsetRad,
  Constants.kFrontRightDriveAbsoluteEncoderReversed);
// defines back left swerve drive module
  private final SwerveModule backLeft = new SwerveModule(
  Constants.kBackLeftDriveMotorPort,
  Constants.kBackLeftTurningDrivePort,
  Constants.kBackLeftDriveEncoderReversed,
  Constants.kBackLeftTurningEncoderRevered,
  Constants.kBackLeftDriveAbsoluteEncoderPort,
  Constants.kBackLeftDriveAbsoluteEncoderOffsetRad,
  Constants.kBackLeftDriveAbsoluteEncoderReversed);
// defines back right swerve drive module
  private final SwerveModule backRight = new SwerveModule(
  Constants.kBackRightDriveMotorPort,
  Constants.kBackRightTurningDrivePort,
  Constants.kBackRightDriveEncoderReversed,
  Constants.kBackRightTurningEncoderRevered,
  Constants.kBackRightDriveAbsoluteEncoderPort,
  Constants.kBackRightDriveAbsoluteEncoderOffsetRad,
  Constants.kBackRightDriveAbsoluteEncoderReversed);
// allows access to gyro module
  private AnalogGyro gyro = new AnalogGyro(0);

  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.kDriveKinematics, 
  getRotation2d(), null, getPose());
  
 

  /** Creates a new SwerveSubSystem. */
  public SwerveSubSystem() 
  {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
      } catch (Exception e) 
      {

      }
    }).start();
  }
// reinitializes gyro
  public void zeroHeading()
  {
    gyro.reset();
  }
// retrieves heading of robot from gyro
public double getHeading()
{
  return Math.IEEEremainder(gyro.getAngle(), 360);
}
// translates heading into robot speak
public Rotation2d getRotation2d()
{
  return Rotation2d.fromDegrees(getHeading());
}

public Pose2d getPose() 
{
  return odometer.getPoseMeters();
}

public void resetOdometry(Pose2d pose) 
{
  odometer.resetPosition(getRotation2d(), null, pose);
  
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Robot Heading: ", getHeading());

  }
// function to stop all motors in drive
  public void stopModules()
  {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }
// returns robot to default state
  public void setModuleStates(SwerveModuleState[] desiredStates)
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kMaxSpeedMetersperSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }
}
