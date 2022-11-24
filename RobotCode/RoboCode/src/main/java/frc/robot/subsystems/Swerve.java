// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Swerve extends SubsystemBase {
  /** Creates a new Swerve. */

  private boolean isCalibrating;
  private boolean offSetCalibration = true;
  private boolean driveCalibration = false;
  private boolean rotCalibration = true;

  private Rotation2d fieldAngel = new Rotation2d();
  private final Field2d field2d = new Field2d();

  private AHRS gyroAhrs = new AHRS();

  TalonFX driveMotorBL = new TalonFX(11);

  private double[] pidValues = {
    0.09698,
    0.09698,
    0.09698,
    0.09698
  };

  final boolean invertAllModules = false;
  private SwerveModule[] modules = new SwerveModule[]{
new SwerveModule("FL", new TalonFX(17), new TalonFX(13), new DutyCycleEncoder( new DigitalInput(0)), Rotation2d.fromDegrees(-27), true^invertAllModules, new PIDController(pidValues[0], 0, 0)), //! Front Left
    new SwerveModule("FR", new TalonFX(14), new TalonFX(15), new DutyCycleEncoder( new DigitalInput(2)), Rotation2d.fromDegrees(-128), true^invertAllModules, new PIDController(pidValues[1], 0, 0)), //! Front Right
    new SwerveModule("RL", driveMotorBL, new TalonFX(16), new DutyCycleEncoder(new DigitalInput(1)), Rotation2d.fromDegrees(54), 
    false^invertAllModules, new PIDController(pidValues[2], 0, 0)), //! Back Left
  };


  public Swerve() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
