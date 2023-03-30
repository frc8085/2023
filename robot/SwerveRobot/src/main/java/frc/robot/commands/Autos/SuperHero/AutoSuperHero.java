// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.SuperHero;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Autos.Shared.LockWheelsAtEndOfAuto;
import frc.robot.commands.Autos.Shared.Move.AutoMoveOnChargeStationFromFront;
import frc.robot.commands.Autos.Shared.Move.AutoMoveToFrontChargeStation;
import frc.robot.commands.Autos.Shared.Move.AutoMoveToPickup;
import frc.robot.commands.Autos.Shared.Move.AutoPickupCargoClean;
import frc.robot.commands.Autos.Shared.Move.AutoPickupConeClean;
import frc.robot.commands.Autos.Shared.ScoreHigh.AutoScoreHighCone;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoSuperHero extends SequentialCommandGroup {
  public AutoSuperHero(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    addCommands(
        new ParallelRaceGroup(
            new SequentialCommandGroup(
                // 1. Score Cone
                new AutoScoreHighCone(m_drive, m_altitude, m_extension, m_intake),
                // 2. Move to pickup cargo position
                new AutoMoveToPickup(m_drive, m_altitude, m_extension),
                // 3. Intake down and pickup cargo
                new AutoPickupConeClean(m_drive, m_altitude, m_extension, m_intake),
                // 4. Intake up and move to charge station
                new AutoMoveToFrontChargeStation(m_drive, m_altitude, m_extension, m_intake),
                // 5. Move on Charge Station
                new AutoMoveOnChargeStationFromFront(m_drive, m_altitude, m_extension)),
            new LockWheelsAtEndOfAuto(m_drive))
    // 6. Rotate so camera can see apriltags

    );
  }

}