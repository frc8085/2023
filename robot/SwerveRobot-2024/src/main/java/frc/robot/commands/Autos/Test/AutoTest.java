// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DropCone;
import frc.robot.commands.Autos.Shared.AutoResetOdometry;
import frc.robot.commands.Autos.Shared.Balance.AprilTagBalance;
import frc.robot.commands.Autos.Shared.Balance.AutoFinalBalance;
import frc.robot.commands.Autos.Shared.Move.AutoTravelToPickupAndStartLoweringIntakeClean;
import frc.robot.commands.Autos.Shared.Move.AutoTravelToPickupClean;
import frc.robot.commands.Autos.Shared.ScoreHigh.AutoScoreHighCone;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoTest extends SequentialCommandGroup {
  public AutoTest(
      DriveSubsystem m_drive, Altitude m_altitude, Extension m_extension, Intake m_intake) {
    addCommands(new AutoScoreHighCone(m_drive, m_altitude, m_extension, m_intake));
  }

}