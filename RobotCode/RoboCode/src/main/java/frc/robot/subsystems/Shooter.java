// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
/*
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;*/

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  WPI_TalonFX masterMotor = new WPI_TalonFX(Constants.MASTERMOTOR_ID);
  WPI_TalonFX slaveMotor = new WPI_TalonFX(Constants.SLAVEMOTOR_ID);
/*
  private double kP = 0.11533;
  private double kI = 0.0;
  private double kD = 0.0;

  private double kS = 0.74791;
  private double kV = 0.11447;
  private double kA = 0.0063222;

*/
  
  
  /** Creates a new shooter. */
  public Shooter() {
    slaveMotor.setInverted(false);
    masterMotor.setInverted(true);
    slaveMotor.follow(masterMotor);
  }

  public void shooterShoot(){
    masterMotor.set(ControlMode.PercentOutput, 0.5);
  }
  public void shootStop(){
    masterMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
