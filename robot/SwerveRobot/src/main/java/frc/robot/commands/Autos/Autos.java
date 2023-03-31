package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autos.DynamicDuo.AutoDynamicDuo;
import frc.robot.commands.Autos.DynamicDuo.AutoDynamicDuoNew;
import frc.robot.commands.Autos.Henchman.AutoHenchman;
import frc.robot.commands.Autos.MainCharacter.AutoMainCharacter;
import frc.robot.commands.Autos.MainCharacter.AutoMainCharacterOld;
import frc.robot.commands.Autos.Sidekick.AutoSidekick;
import frc.robot.commands.Autos.Sidekick.AutoSidekickNew;
import frc.robot.commands.Autos.SuperHero.AutoSuperHero;
import frc.robot.commands.Autos.SuperHero.AutoSuperHeroNew;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

public final class Autos {
  public static Alliance m_alliance = Alliance.RED;

  public enum Auto {
    SIDEKICK, SUPERHERO, MAIN_CHARACTER, DYNAMIC_DUO, HENCHMAN, SIDEKICK_NEW, SUPERHERO_NEW, DYNAMIC_DUO_NEW,
    MAIN_CHARACTER_OLD
  }

  public enum Alliance {
    BLUE, RED
  }

  public static Alliance getAlliance() {
    return m_alliance;
  }

  public static String GetAutoName(Auto selected) {
    String name;

    switch (selected) {
      case SIDEKICK:
        name = "(15pt) SIDEKICK: 2x Score High, Leave";
        break;
      case SUPERHERO:
        name = "(21pt) SUPERHERO: 1x Score, Pickup, Balance";
        break;
      case MAIN_CHARACTER:
        name = "(21pt) MAIN CHARACTER: 1x Score, Leave, Balance";
        break;
      case DYNAMIC_DUO:
        name = "(23pt) DYNAMIC DUO: 2x Score High, Leave, Dock";
        break;
      case SIDEKICK_NEW:
        name = "(15pt) SIDEKICK: 2x Score High, Leave - INTAKE MOVES DURING TRAVEL";
        break;
      case SUPERHERO_NEW:
        name = "(21pt) SUPERHERO: 1x Score, Pickup, Balance - INTAKE MOVES DURING TRAVEL";
        break;
      case MAIN_CHARACTER_OLD:
        name = "(21pt) MAIN CHARACTER: 1x Score, Leave, Balance - WATERBURY CODE";
        break;
      case DYNAMIC_DUO_NEW:
        name = "(23pt) DYNAMIC DUO: 2x Score High, Leave, Dock - INTAKE MOVES DURING TRAVEL";
        break;
      case HENCHMAN:
        name = "(15pt) HENCHMAN: 2x Score High, Leave - DIRTY SIDE";
        break;
      default:
        throw new AssertionError("Illegal value" + selected);
    }
    return name;
  }

  public static Command SelectAuto(Auto selected, Alliance alliance, DriveSubsystem m_drive, Altitude m_altitude,
      Extension m_extension, Intake m_intake) {
    Command autoCommand;
    m_alliance = alliance;

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
      case SIDEKICK_NEW:
        autoCommand = new AutoSidekickNew(m_drive, m_altitude, m_extension, m_intake);
        break;
      case SUPERHERO_NEW:
        autoCommand = new AutoSuperHeroNew(m_drive, m_altitude, m_extension, m_intake);
        break;
      case MAIN_CHARACTER_OLD:
        autoCommand = new AutoMainCharacterOld(m_drive, m_altitude, m_extension, m_intake);
        break;
      case DYNAMIC_DUO_NEW:
        autoCommand = new AutoDynamicDuoNew(m_drive, m_altitude, m_extension, m_intake);
        break;
      case HENCHMAN:
        autoCommand = new AutoHenchman(m_drive, m_altitude, m_extension, m_intake);
        break;
      default:
        throw new AssertionError("Illegal value" + selected);
    }

    return autoCommand;

  }

}