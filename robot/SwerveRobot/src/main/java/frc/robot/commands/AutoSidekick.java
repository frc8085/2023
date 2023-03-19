// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

/** An example command that uses an example subsystem. */
public class AutoSidekick extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Intake m_intake;
    private final DriveSubsystem m_drive;
    private final Altitude m_altitude;
    private final Extension m_extension;

    /**
     * Creates a new AutoSidekick.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AutoSidekick(DriveSubsystem drive, Altitude altitude, Extension extension, Intake intake) {
        m_drive = drive;
        m_altitude = altitude;
        m_extension = extension;
        m_intake = intake;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_drive, m_altitude, m_extension, m_intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_drive.resetOdometry(new Pose2d());
        m_drive.zeroHeading();

        Commands.sequence(
                new MoveToMidConeDropOff(m_extension, m_altitude));
    }

    // 1. score
    // 2. drive back 3m & turn 180
    // 3. move tontake down
    // 4. drive forward 2 meters
    // 5. travel position
    // 6. drive to 1 turn 180
    // 7. shoot

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
