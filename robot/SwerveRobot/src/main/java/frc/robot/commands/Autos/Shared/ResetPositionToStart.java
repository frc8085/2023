// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Shared;

import frc.robot.subsystems.Altitude;
import frc.robot.subsystems.Extension;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Move until limit switches are hit and then stop */
public class ResetPositionToStart extends CommandBase {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final Altitude m_altitude;
    private final Extension m_extension;
    boolean altitudeReached = false;
    boolean extensionReached = false;

    /**
     * Creates a new ResetPositionToStart.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ResetPositionToStart(Altitude altitude, Extension extension) {
        m_altitude = altitude;
        m_extension = extension;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_altitude, m_extension);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_altitude.raiseAltitude();
        m_extension.retractExtension();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_altitude.stopAltitude();
        m_extension.stopExtension();

        m_altitude.resetAltitudeEncoderAtTopLimit();
        m_extension.resetExtensionEncoderAtRetractionLimit();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        altitudeReached = altitudeReached || m_altitude.isAltitudeTopLimitHit();
        extensionReached = extensionReached || m_extension.isRetractionLimitHit();

        return altitudeReached && extensionReached;
    }
}
