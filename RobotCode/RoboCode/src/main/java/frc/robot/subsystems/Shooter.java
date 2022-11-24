// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  WPI_TalonFX masterMotor = new WPI_TalonFX(Constants.MASTERMOTOR_ID);
  WPI_TalonFX slaveMotor = new WPI_TalonFX(Constants.SLAVEMOTOR_ID);

  double shooterCurrentRPM;
  double PIDOutput;
  double feedForwardOutput;


  private double kP = 0.11533;
  private double kI = 0.0;
  private double kD = 0.0;

  private double kS = 0.74791;
  private double kV = 0.11447;
  private double kA = 0.0063222;

  PIDController pid = new PIDController(kP, kI, kD);
 
  SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV, kA);

  double error;
  double output;
  
  /** Creates a new shooter. */
  public Shooter() {
    slaveMotor.setInverted(false);
    masterMotor.setInverted(true);
    slaveMotor.follow(masterMotor);
  }
  public double getRPM(){
    double rawSensorData = masterMotor.getSelectedSensorVelocity();
   // shooterCurrentRPM = (rawSensorData * 10.)/2048. * 60.;
    shooterCurrentRPM = (rawSensorData /2048. *60.);
    return shooterCurrentRPM;

  }
  public void setRPM(int RPM){
    pid.setTolerance(100);
    PIDOutput = pid.calculate(shooterCurrentRPM/60., RPM/60.);
    //PIDOutput = pid.calculate(shooterCurrentRPM, RPM);

    
    feedForwardOutput = feedForward.calculate(RPM/60.);

    output = (PIDOutput + feedForwardOutput)/ RobotController.getBatteryVoltage();
    masterMotor.set(ControlMode.PercentOutput, output); 
  }
    

  public void shootStop(){
    masterMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
