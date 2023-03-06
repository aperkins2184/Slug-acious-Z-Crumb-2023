// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogEncoder;

import frc.robot.Constants;

//import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;


  /** Creates a new SwerveModule. */
  public class SwerveModule {
    //Declare turning and driver motors
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;
    //Declare drive and turing Encoders
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;
    //Declare PID controller
    private final PIDController turningPidController;
    // Create an absolute controller
    private final AnalogEncoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed)    
    {

      this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
      this.absoluteEncoderReversed = absoluteEncoderReversed;
      absoluteEncoder = new AnalogEncoder(absoluteEncoderId);
        
    // defines types of motors
    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
    // defines inverted motors
    driveMotor.setInverted(turningMotorReversed);
    turningMotor.setInverted(turningMotorReversed);
    // defines encoders
    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();
    // establishes speed limits for motors
    driveEncoder.setPositionConversionFactor(Constants.kDriveEncoderRot2Meter);
    driveEncoder.setVelocityConversionFactor(Constants.kDriveEncoderRPM2MeterPerSec);
    turningEncoder.setPositionConversionFactor(Constants.kTurningEncoderRot2Rad);
    turningEncoder.setVelocityConversionFactor(Constants.kTurningEncoderRPM2RadPerSec);
    // defines PID motor
    turningPidController = new PIDController(Constants.kPTurning, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
    // resets everything to 0 upon initialization
    resetEncoders();
  }
   // identifies position of drive motors
  public double getDrivePosition()
  {
    return driveEncoder.getPosition();
  }
  // identifies positions of rotation motors
  public double getTurningPosition()
  {
    return turningEncoder.getPosition();
  }
  // tachometer for drive motors
  public double getDriveVelocity()
  {
    return driveEncoder.getVelocity();
  }
  // tachometer for rotation motors
  public double getTurningVelocity()
  {
    return turningEncoder.getVelocity();
  }
  // converts radians to coordinates
  public double getAbsoluteEncoderRad()
  {
    double angle = absoluteEncoder.getAbsolutePosition() / RobotController.getVoltage5V();
    angle *= 2.0 * Math.PI;
    angle -= absoluteEncoderOffsetRad;
    return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
  }
  // function for resetting encoders
  public void resetEncoders()
  {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }
  // resets position of rotation motors to 0 upon initialization
  public SwerveModuleState getState()
  {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }
  // synchronizes wheels
  public void setDesiredState(SwerveModuleState state)
  {
    if (Math.abs(state.speedMetersPerSecond) < 0.001)
    {
      stop();
      return;
    }
    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / Constants.kMaxSpeedMetersperSecond);
    turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() +"] state", state.toString());
  
  }
// stops motors when not in use
 public void stop()
 {
  driveMotor.set(0);
  turningMotor.set(0);
 }
}
 /*
@Override
public void setDefaultCommand(Command defaultCommand) {
    
}  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
   }  
  }
  */
