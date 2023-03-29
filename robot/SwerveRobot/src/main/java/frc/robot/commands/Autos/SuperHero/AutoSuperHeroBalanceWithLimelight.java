// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SuperHero;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoTurnToDegreeGyro;
import frc.robot.commands.Autos.Shared.Move.AutoMoveOnChargeStationFromFront;
import frc.robot.commands.Autos.Shared.Move.AutoMoveToFrontChargeStation;
import frc.robot.commands.Autos.Shared.Move.AutoMoveToPickup;
import frc.robot.commands.Autos.Shared.Move.AutoPickupCubeClean;
import frc.robot.commands.Autos.Shared.Move.AutoPickupConeClean;
import frc.robot.commands.Autos.Shared.ScoreHigh.AutoScoreHighCone;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoSuperHeroBalanceWithLimelight extends SequentialCommandGroup {
  public AutoSuperHeroBalanceWithLimelight(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        // turn to face limelight to grid
        new AutoTurnToDegreeGyro(180, m_drive, false)
    // position to center of charge station based on limelight

    );
  }

}