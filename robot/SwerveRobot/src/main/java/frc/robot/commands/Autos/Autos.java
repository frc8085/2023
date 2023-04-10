package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ResetPositionToStart;
import frc.robot.commands.Autos.DynamicDuo.AutoDynamicDuo;
import frc.robot.commands.Autos.DynamicDuo.AutoDynamicDuoNew;
import frc.robot.commands.Autos.Henchman.AutoHenchman;
import frc.robot.commands.Autos.Henchman.AutoHenchmanCone;
import frc.robot.commands.Autos.MainCharacter.AutoMainCharacter;
import frc.robot.commands.Autos.MainCharacter.AutoMainCharacterOld;
import frc.robot.commands.Autos.Sidekick.AutoSidekick;
import frc.robot.commands.Autos.Sidekick.AutoSidekickCone;
import frc.robot.commands.Autos.Sidekick.AutoSidekickNew;
import frc.robot.commands.Autos.SuperHero.AutoSuperHero;
import frc.robot.commands.Autos.SuperHero.AutoSuperHeroNew;
import frc.robot.commands.Autos.Test.AutoTest;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public final class Autos {
  public static Alliance m_alliance = Alliance.RED;
  private LimelightSubsystem m_limelight;

  public enum Auto {
    SIDEKICK, SUPERHERO, MAIN_CHARACTER, DYNAMIC_DUO, HENCHMAN, SIDEKICK_CONE, SUPERHERO_NEW,
    HENCHMAN_CONE, DYNAMIC_DUO_NEW,
    MAIN_CHARACTER_OLD, TEST
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
      case SIDEKICK_CONE:
        name = "(15pt) SIDEKICK: 2x Score High, Leave - PICKUP CONE";
        break;
      case SUPERHERO_NEW:
        name = "(21pt) SUPERHERO: 1x Score, Pickup, Balance - INTAKE MOVES DURING TRAVEL";
        break;
      case MAIN_CHARACTER_OLD:
        name = "(21pt) MAIN CHARACTER: 1x Score, Leave, Balance - OLD";
        break;
      case DYNAMIC_DUO_NEW:
        name = "(23pt) DYNAMIC DUO: 2x Score High, Leave, Dock - INTAKE MOVES DURING TRAVEL";
        break;
      case HENCHMAN:
        name = "(15pt) HENCHMAN: 2x Score High, Leave - DIRTY SIDE";
        break;
      case HENCHMAN_CONE:
        name = "(15pt) HENCHMAN CONE: 2x Score High, Leave - DIRTY SIDE";
        break;
      case TEST:
        name = "TEST";
        break;
      default:
        throw new AssertionError("Illegal value" + selected);
    }
    return name;
  }

  public static Command SelectAuto(Auto selected, Alliance alliance, DriveSubsystem m_drive, Altitude m_altitude,
      Extension m_extension, Intake m_intake, LimelightSubsystem limelight) {
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
      case SIDEKICK_CONE:
        autoCommand = new AutoSidekickCone(m_drive, m_altitude, m_extension, m_intake);
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
      case HENCHMAN_CONE:
        autoCommand = new AutoHenchmanCone(m_drive, m_altitude, m_extension, m_intake);
        break;
      case TEST:
        autoCommand = new AutoTest(m_drive, m_altitude, m_extension, m_intake, limelight);
        break;
      default:
        throw new AssertionError("Illegal value" + selected);
    }

    Command reset = new ResetPositionToStart(m_altitude, m_extension);

    return reset.andThen(autoCommand);

  }

}