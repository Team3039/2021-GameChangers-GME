/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ActuateIntake;
import frc.robot.commands.SetHopperFeedingMode;
import frc.robot.commands.SetHopperIntakingMode;
import frc.robot.commands.SetIntakeSpeed;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class IndexCells extends SequentialCommandGroup {
  /**
   * Creates a new IndexCells.
   */
  public IndexCells() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new ActuateIntake(false),
          new SetIntakeSpeed(0),
          new SetHopperIntakingMode());
  }
}
