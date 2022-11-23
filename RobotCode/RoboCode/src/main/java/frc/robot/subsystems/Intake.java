// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

public WPI_TalonSRX intakeMotor = new WPI_TalonSRX(Constants.INTAKEMOTOR_ID);

  public Intake() {
    intakeMotor.setInverted(false);
  }
  public void intakeIn(){
    intakeMotor.set(ControlMode.PercentOutput, 0.5);
  }
  public void intakeOut(){
    intakeMotor.set(ControlMode.PercentOutput, -0.5);
  }
  public void intakeStop(){
    intakeMotor.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
