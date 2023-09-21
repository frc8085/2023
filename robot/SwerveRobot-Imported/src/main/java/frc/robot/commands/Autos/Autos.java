package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Autos.DynamicDuo.AutoDynamicDuoMoveToChargeStation;
import frc.robot.commands.Autos.DynamicDuo.AutoDynamicDuoReturnToScore;
import frc.robot.commands.Autos.DynamicDuo.AutoDynamicDuoSecondScore;
import frc.robot.commands.Autos.Shared.Move.AutoMoveToPickup;
import frc.robot.commands.Autos.Shared.Move.AutoPickupCargo;
import frc.robot.commands.Autos.Shared.ScoreHigh.AutoScoreHighCone;
import frc.robot.commands.Autos.Sidekick.AutoSidekickReturnToScore;
import frc.robot.commands.Autos.Sidekick.AutoSidekickSecondScore;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

public final class Autos {

  /** An example command that uses an example subsystem. */

  public static CommandBase AutoDynamicDuo(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {

    return Commands.sequence(
        // new ParallelCommandGroup(
        // Reset Heading & Odometry
        // new AutoResetOdometry(m_drive),
        // 1. Score Cone
        new AutoScoreHighCone(m_drive, m_altitude, m_extension, m_intake),
        // 2. Move to pickup cargo position
        new AutoMoveToPickup(m_drive, m_altitude, m_extension),
        // 3. Intake down and pickup cargo
        new AutoPickupCargo(m_drive, m_altitude, m_extension, m_intake),
        // 4. Intake up and move back to grid
        new AutoDynamicDuoReturnToScore(m_drive, m_altitude, m_extension, m_intake),
        // 5. Shoot cube
        new AutoDynamicDuoSecondScore(m_altitude, m_extension, m_intake),
        // 6. Move to Charge Station
        new AutoDynamicDuoMoveToChargeStation(m_drive));
  }

  public static CommandBase AutoSidekick(
      DriveSubsystem m_drive,
      Altitude m_altitude,
      Extension m_extension,
      Intake m_intake) {
    return Commands.sequence(
        // new ParallelCommandGroup(
        // Reset Heading & Odometry
        // new AutoResetOdometry(m_drive),
        // 1. Score Cone
        new AutoScoreHighCone(m_drive, m_altitude, m_extension, m_intake),
        // 2. Move to pickup cargo position
        new AutoMoveToPickup(m_drive, m_altitude, m_extension),
        // 3. Intake down and pickup cargo
        new AutoPickupCargo(m_drive, m_altitude, m_extension, m_intake),
        // 4. Intake up and move back to grid
        new AutoSidekickReturnToScore(m_drive, m_altitude, m_extension, m_intake),
        // 5. Shoot cube
        new AutoSidekickSecondScore(m_altitude, m_extension, m_intake));
  }

}