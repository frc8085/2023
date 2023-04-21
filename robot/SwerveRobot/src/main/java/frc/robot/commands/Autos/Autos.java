package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.TuningModeConstants;
import frc.robot.commands.ResetPositionToStart;
import frc.robot.commands.Autos.DynamicDuo.AutoDynamicDuoNew;
import frc.robot.commands.Autos.Henchman.AutoHenchman;
import frc.robot.commands.Autos.Henchman.AutoHenchmanCone;
import frc.robot.commands.Autos.MainCharacters.AutoMainCharacters;
import frc.robot.commands.Autos.MainCharacters.AutoMainCharactersTurning;
import frc.robot.commands.Autos.Shared.Balance.AutoDecreasingSpeedBalance;
import frc.robot.commands.Autos.Shared.Balance.AutoConstantSpeedFinalBalance;
import frc.robot.commands.Autos.Shared.Balance.AutoPitchRollBalance;
import frc.robot.commands.Autos.Sidekick.AutoSidekick;
import frc.robot.commands.Autos.Sidekick.AutoSidekickCone;
import frc.robot.commands.Autos.SuperHero.AutoSuperHero;
import frc.robot.commands.Autos.Test.AutoTest;
import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;

public final class Autos {
  public static Alliance m_alliance = Alliance.RED;

  private boolean TESTING_BALANCE = TuningModeConstants.kBalanceTuning;

  public enum Auto {
    CHIDI,
    SIDEKICK,
    SUPERHERO,
    DYNAMIC_DUO,
    HENCHMAN,
    SIDEKICK_CONE,
    HENCHMAN_CONE,
    TEST
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
      case CHIDI:
        name = "(21pt) CHIDI: Decreasing Speed Balance Main Char";
        break;
      case SIDEKICK:
        name = "(15pt) SIDEKICK: 2x Score High, Leave";
        break;
      case SUPERHERO:
        name = "(21pt) SUPERHERO: 1x Score, Pickup, Balance";
        break;
      case DYNAMIC_DUO:
        name = "(23pt) DYNAMIC DUO: 2x Score High, Leave, Dock - INTAKE MOVES DURING TRAVEL";
        break;
      case HENCHMAN:
        name = "(15pt) HENCHMAN: 2x Score High, Leave - DIRTY SIDE";
        break;
      case SIDEKICK_CONE:
        name = "(15pt) SIDEKICK CONE: 2x Score High, Leave - PICKUP CONE";
        break;
      case HENCHMAN_CONE:
        name = "(15pt) HENCHMAN CONE: 2x Score High, Leave - DIRTY SIDE CONE";
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
      Extension m_extension, Intake m_intake) {
    Command autoCommand;
    m_alliance = alliance;

    switch (selected) {
      case CHIDI:
        autoCommand = new AutoMainCharacters(m_drive, m_altitude, m_extension, m_intake,
            new AutoDecreasingSpeedBalance(m_drive));
        break;

      case SIDEKICK:
        autoCommand = new AutoSidekick(m_drive, m_altitude, m_extension, m_intake);
        break;
      case SUPERHERO:
        autoCommand = new AutoSuperHero(m_drive, m_altitude, m_extension, m_intake);
        break;
      case DYNAMIC_DUO:
        autoCommand = new AutoDynamicDuoNew(m_drive, m_altitude, m_extension, m_intake);
        break;

      case HENCHMAN:
        autoCommand = new AutoHenchman(m_drive, m_altitude, m_extension, m_intake);
        break;
      case SIDEKICK_CONE:
        autoCommand = new AutoSidekickCone(m_drive, m_altitude, m_extension, m_intake);
        break;

      case HENCHMAN_CONE:
        autoCommand = new AutoHenchmanCone(m_drive, m_altitude, m_extension, m_intake);
        break;
      // case AUTOSCOREONLY:
      // autoCommand = new AutoScoreOnly(m_drive, m_altitude, m_extension, m_intake);
      // break;
      case TEST:
        autoCommand = new AutoTest(m_drive, m_altitude, m_extension, m_intake);
        break;
      default:
        throw new AssertionError("Illegal value" + selected);
    }

    Command reset = new ResetPositionToStart(m_altitude, m_extension);

    return reset.andThen(autoCommand);

  }

}