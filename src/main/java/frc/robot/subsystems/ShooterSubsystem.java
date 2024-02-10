// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private final CANSparkMax shooterMotor = new CANSparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless) ;
  private final CANSparkMax indexMotor = new CANSparkMax(Constants.ShooterConstants.INDEX_MOTOR_ID, MotorType.kBrushless) ; 
  

  public ShooterSubsystem() {
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.getPIDController().setP(0);
    shooterMotor.getPIDController().setI(0);
    shooterMotor.getPIDController().setD(0);
    shooterMotor.getPIDController().setFF(0);
    shooterMotor.getPIDController().setSmartMotionAllowedClosedLoopError(100, 0);

    indexMotor.setInverted (true);
    indexMotor.setOpenLoopRampRate(Constants.ShooterConstants.INDEX_RAMP_RATE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 
}
