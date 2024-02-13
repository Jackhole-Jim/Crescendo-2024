// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AmpWhipperSubsystem extends SubsystemBase {

  private final PWM linearActuatorLeft = new PWM(0);
  private final PWM linearActuatorRight = new PWM(1);

  /** Creates a new AmpWhipperSubsystem. */
  public AmpWhipperSubsystem() {
    linearActuatorLeft.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    linearActuatorRight.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command extendActuators(){
    return runOnce(() -> {
      linearActuatorLeft.setSpeed(1);
      linearActuatorRight.setSpeed(1);
    });
  }

  public Command retractActuators(){
    return runOnce(() -> {
      linearActuatorLeft.setSpeed(-1);
      linearActuatorRight.setSpeed(-1);
    });
  }
}
