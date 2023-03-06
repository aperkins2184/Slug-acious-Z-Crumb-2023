// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubSystem;

public class ControlerCommand extends CommandBase {
  /** Creates a new ControlerCommand. */

    // Use addRequirements() here to declare subsystem dependencies.
    private final SwerveSubSystem swerveSubSystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    // module for each complete 'wheel' of swerve drive
    public ControlerCommand(SwerveSubSystem swerveSubSystem,
    Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
    Supplier<Boolean> fieldOrientedFunction)
    {
      this.swerveSubSystem = swerveSubSystem;
      this.xSpdFunction = xSpdFunction;
      this.ySpdFunction = ySpdFunction;
      this.turningSpdFunction = turningSpdFunction;
      this.fieldOrientedFunction = fieldOrientedFunction;
      this.xLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.yLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAccelerationUnitsPerSecond);
      this.turningLimiter = new SlewRateLimiter(Constants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
      addRequirements(swerveSubSystem);
    }
  

 


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    // Deadband setting to eliminate unintentional motor running
    xSpeed = Math.abs(xSpeed) > Constants.deadBand ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > Constants.deadBand ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > Constants.deadBand ? turningSpeed : 0.0;
    // Smooth out controls by adjusting
    xSpeed = xLimiter.calculate(xSpeed) * Constants.kTeleDriveMaxSpeedMetersPerSecond;
    ySpeed = yLimiter.calculate(ySpeed) * Constants.kTeleDriveMaxSpeedMetersPerSecond;
    turningSpeed = turningLimiter.calculate(turningSpeed)
            * Constants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

    // Build Robot Speeds
    ChassisSpeeds robotSpeeds;
    if (fieldOrientedFunction.get())
    {
      robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds (
        xSpeed, ySpeed, turningSpeed, swerveSubSystem.getRotation2d()
      );
    }
    else 
    {
      robotSpeeds =new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }
     // Convert chassis speeds to individual module states     
    SwerveModuleState[] moduleStates = Constants.kDriveKinematics.toSwerveModuleStates(robotSpeeds);
    // Output module state to the wheels
    swerveSubSystem.setModuleStates(moduleStates);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    swerveSubSystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
