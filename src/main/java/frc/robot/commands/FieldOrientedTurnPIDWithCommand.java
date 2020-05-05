/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsytem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FieldOrientedTurnPIDWithCommand extends PIDCommand {
  /**
   * Creates a new FieldOrientedTurnPIDWithCommand.
   */
  public FieldOrientedTurnPIDWithCommand(DriveSubsytem m_drive,double targetAngle) {
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.turnP,DriveConstants.turnI, DriveConstants.turnD),
        // This should return the measurement
        () -> m_drive.getHeading(),
        // This should return the setpoint (can also be a constant)
        targetAngle,
        // This uses the output
        output -> {
          // Use the output here
          m_drive.arcadeDrive(0, output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(m_drive);
    getController().setTolerance(DriveConstants.turnAccuracy);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
