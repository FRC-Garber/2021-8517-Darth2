// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDefaultDummyCommandGroup extends SequentialCommandGroup {
  /** Creates a new AutoDefaultDummyCommandGroup. */
  public AutoDefaultDummyCommandGroup() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //new DriveAutoMovePIDCommand(48.0),
      new ChassisAutoRotatePIDCommand(90)
      // new DriveAutoRotatePIDCommand(90, 1),
      // new DriveAutoMovePIDCommand(-48.0,1),
      // new DriveAutoRotatePIDCommand(-90, 1)
    );
  }
}
