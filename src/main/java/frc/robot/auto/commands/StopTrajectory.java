package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class StopTrajectory extends CommandBase {

    private static final Drive mDrive = Drive.getInstance();

    @Override
    public void initialize() {
        System.out.println("Stop Trajectory Started");
        mDrive.tankDriveVolts(0,0);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Stop Trajectory Finished");
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
