// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ClimbCommandDown;
import frc.robot.commands.ClimbCommandUp;
import frc.robot.commands.IntakeCommandIn;
import frc.robot.commands.IntakeCommandOut;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //Subsystems
 Climb climbcik = new Climb();
 Intake intake = new Intake();

 //Commands
 IntakeCommandIn ball_in = new IntakeCommandIn(intake); 
 IntakeCommandOut ball_out = new IntakeCommandOut(intake);
 ClimbCommandUp top_yukari = new ClimbCommandUp(climbcik);
 ClimbCommandDown top_asagi = new ClimbCommandDown(climbcik);

 //Kumanda
 XboxController operatoryanki = new XboxController(Constants.OPERATOR_ID);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton climbButtonUp = new JoystickButton(operatoryanki, 1);
    climbButtonUp.whileHeld(top_yukari);

    JoystickButton climbButtonDown = new JoystickButton(operatoryanki, 2);
    climbButtonDown.whileHeld(top_asagi);

    JoystickButton intakeButtonIn = new JoystickButton(operatoryanki, 3);
    intakeButtonIn.whileHeld(ball_in);

    JoystickButton intakeButtonOut = new JoystickButton(operatoryanki, 4);
    intakeButtonOut.whileHeld(ball_out);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
