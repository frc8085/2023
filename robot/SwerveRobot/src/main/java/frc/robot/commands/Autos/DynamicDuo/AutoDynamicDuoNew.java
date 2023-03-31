// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.DynamicDuo;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.Shared.Move.AutoTravelToPickupAndStartLoweringIntakeClean;
import frc.robot.commands.Autos.Shared.Move.AutoTravelToPickupClean;
import frc.robot.commands.Autos.Shared.Move.AutoTravelAndPickupCubeClean;
import frc.robot.commands.Autos.Shared.ScoreHigh.AutoScoreHighCone;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

/** An example command that uses an example subsystem. */
public class AutoDynamicDuoNew extends SequentialCommandGroup {
    public AutoDynamicDuoNew(
            DriveSubsystem m_drive,
            Altitude m_altitude,
            Extension m_extension,
            Intake m_intake) {
        addCommands(
                // new ParallelCommandGroup(
                // new ParallelCommandGroup(
                // Reset Heading & Odometry
                // new AutoResetOdometry(m_drive),
                // 1. Score Cone
                new SequentialCommandGroup(
                        new AutoScoreHighCone(m_drive, m_altitude, m_extension, m_intake),
                        // 2. Move to pickup cargo position
                        new AutoTravelToPickupAndStartLoweringIntakeClean(m_drive, m_altitude, m_extension),
                        // 3. Intake down and pickup cargo
                        new AutoTravelAndPickupCubeClean(m_drive, m_altitude, m_extension, m_intake),
                        // 4. Intake up and move back to grid
                        new AutoDynamicDuoReturnToScore(m_drive, m_altitude, m_extension, m_intake),
                        // 5. Shoot cube
                        new AutoDynamicDuoSecondScore(m_altitude, m_extension, m_intake),
                        // 6. Move to Charge Station
                        new AutoDynamicDuoMoveToChargeStation(m_drive),
                        new AutoDynamicDuoMoveOnChargeStation(m_drive),
                        new RunCommand(m_drive::lock, m_drive))
        // 7. Move on Charge Station
        // new AutoDynamicDuoMoveOnChargeStation(m_drive))
        );

    }

}