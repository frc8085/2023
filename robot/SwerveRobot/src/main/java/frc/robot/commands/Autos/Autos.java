package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autos.DynamicDuo.AutoDynamicDuo;
import frc.robot.commands.Autos.MainCharacter.AutoMainCharacter;
import frc.robot.commands.Autos.Sidekick.AutoSidekick;
import frc.robot.commands.Autos.SuperHero.AutoSuperHero;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

public final class Autos {

  public enum Auto {
    SIDEKICK, SUPERHERO, MAIN_CHARACTER, DYNAMIC_DUO
  }

  public static Command SelectAuto(Auto selected, DriveSubsystem m_drive, Altitude m_altitude,
      Extension m_extension, Intake m_intake) {
    Command autoCommand;

    switch (selected) {
      case SIDEKICK:
        autoCommand = new AutoSidekick(m_drive, m_altitude, m_extension, m_intake);
        break;
      case SUPERHERO:
        autoCommand = new AutoSuperHero(m_drive, m_altitude, m_extension, m_intake);
        break;
      case MAIN_CHARACTER:
        autoCommand = new AutoMainCharacter(m_drive, m_altitude, m_extension, m_intake);
        break;
      case DYNAMIC_DUO:
        autoCommand = new AutoDynamicDuo(m_drive, m_altitude, m_extension, m_intake);
        break;
      default:
        throw new AssertionError("Illegal value" + selected);
    }

    return autoCommand;

  }

}