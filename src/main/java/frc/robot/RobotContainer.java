// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import frc.robot.commands.AutoBarrelRacingCommandGroup;
import frc.robot.commands.AutoBounceCommandGroup;
import frc.robot.commands.AutoDefaultDummyCommandGroup;
import frc.robot.commands.AutoGalacticSearchBlue1CommandGroup;
import frc.robot.commands.AutoGalacticSearchBlue2CommandGroup;
import frc.robot.commands.AutoGalacticSearchRed1CommandGroup;
import frc.robot.commands.AutoGalacticSearchRed2CommandGroup;
import frc.robot.commands.AutoSlalomCommandGroup;
import frc.robot.commands.ChassisDefaultCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.LoaderDefaultCommand;
import frc.robot.commands.ShooterDefaultCommand;
import frc.robot.subsystems.ChassisSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LoaderSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final ChassisSubsystem chassisSubsystem = new ChassisSubsystem();
  public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  public static final LoaderSubsystem loaderSubsystem = new LoaderSubsystem();
  public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  public static final Joystick stickDriver = new Joystick(0);
  public static final Joystick stickOperator = new Joystick(1);

  SendableChooser<CommandGroupBase> autoChoice = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    chassisSubsystem.setDefaultCommand(new ChassisDefaultCommand());
    intakeSubsystem.setDefaultCommand(new IntakeDefaultCommand());
    loaderSubsystem.setDefaultCommand(new LoaderDefaultCommand());
    shooterSubsystem.setDefaultCommand(new ShooterDefaultCommand());

    autoChoice.addOption("Galactic Search Red 1", new AutoGalacticSearchRed1CommandGroup());
    autoChoice.addOption("Galactic Search Red 2", new AutoGalacticSearchRed2CommandGroup());
    autoChoice.addOption("Galactic Search Blue 1", new AutoGalacticSearchBlue1CommandGroup());
    autoChoice.addOption("Galactic Search Blue 2", new AutoGalacticSearchBlue2CommandGroup());
    autoChoice.addOption("Barrel Racing", new AutoBarrelRacingCommandGroup());
    autoChoice.addOption("Bounce", new AutoBounceCommandGroup());
    autoChoice.addOption("Slalom", new AutoSlalomCommandGroup());
    autoChoice.setDefaultOption("Default", new AutoDefaultDummyCommandGroup());

    SmartDashboard.putData(autoChoice);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChoice.getSelected();
  }
}
