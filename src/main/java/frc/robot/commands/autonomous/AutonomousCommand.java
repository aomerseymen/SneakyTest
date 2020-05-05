/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousDrive;
import frc.robot.commands.MoveArm;
import frc.robot.commands.RunLift;
import frc.robot.commands.RunShooter;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsytem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutonomousCommand extends SequentialCommandGroup {
  /**
   * Creates a new AutonomousCommand.
   */
  public AutonomousCommand(LiftSubsystem m_lift, ArmSubsystem m_arm, Shooter m_shooter, DriveSubsytem m_drive) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new RunLift(m_lift, 0.5).alongWith(new MoveArm(m_arm, 0.8)), new RunShooter(m_shooter, 1).withTimeout(3),
        new AutonomousDrive(m_drive, 0.8).withTimeout(3));
  }
}
