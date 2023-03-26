package frc.robot.commands.Autos.Shared.ScoreHigh;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.AltitudeConstants;
import frc.robot.Constants.ExtensionConstants;
import frc.robot.commands.Extend;
import frc.robot.commands.RaiseLower;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;

// Move to High Drop Off position
public class AutoMoveToHighDropOff extends SequentialCommandGroup {
  public AutoMoveToHighDropOff(
      Altitude m_altitude,
      Extension m_extension) {
    addCommands(
        new ParallelCommandGroup(
            new Extend(m_extension, ExtensionConstants.kExtensionPositionAutoHighDropOff),
            new RaiseLower(m_altitude, AltitudeConstants.kAltitudeHighDropOffPosition + .1)));

  }
}