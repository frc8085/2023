// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Field locations for path planning
 */
public final class FieldLandmarks {
    private static boolean PRACTICE = true;
    public static final double BluePickupX = PRACTICE ? 5.28 : 6.5;
    public static final double MidStationY = PRACTICE ? 3.12 : 2.7;

    // Intake always facing toward the grid
    public static final class GridPosition {
        public static final double BlueStartX = 1.85;

        public static final Pose2d BlueCRight = new Pose2d(BlueStartX, 0.48, Rotation2d.fromDegrees(-180));
        public static final Pose2d BlueCCenter = new Pose2d(BlueStartX, 1.05, Rotation2d.fromDegrees(-180));
        public static final Pose2d BlueCLeft = new Pose2d(BlueStartX, 1.61, Rotation2d.fromDegrees(-180));

        public static final Pose2d BlueBRight = new Pose2d(BlueStartX, 2.18, Rotation2d.fromDegrees(-180));
        public static final Pose2d BlueBCenter = new Pose2d(BlueStartX, 2.74, Rotation2d.fromDegrees(-180));
        public static final Pose2d BlueBLeft = new Pose2d(BlueStartX, 3.29, Rotation2d.fromDegrees(-180));

        public static final Pose2d BlueARight = new Pose2d(BlueStartX, 3.87, Rotation2d.fromDegrees(-180));
        public static final Pose2d BlueACenter = new Pose2d(BlueStartX, 4.42, Rotation2d.fromDegrees(-180));
        public static final Pose2d BlueALeft = new Pose2d(BlueStartX, 4.96, Rotation2d.fromDegrees(-180));
    }

    // Pickup positions. Intake always facing toward
    public static final class PickupPosition {
        public static final double BluePickupX = PRACTICE ? 5.28 : 6.5;
        public static final Pose2d Blue1 = new Pose2d(BluePickupX, 4.59, Rotation2d.fromDegrees(0));
        public static final Pose2d Blue2 = new Pose2d(BluePickupX, 3.35, Rotation2d.fromDegrees(0));
        public static final Pose2d Blue3 = new Pose2d(BluePickupX, 2.13, Rotation2d.fromDegrees(0));
        public static final Pose2d Blue4 = new Pose2d(BluePickupX, 0.91, Rotation2d.fromDegrees(0));
    }

    // Endpoints for segemented trajectories
    // Intake facing grid
    public static final class SegmentEndpoints {
        public static final Pose2d ApproachBlue1 = new Pose2d(BluePickupX - 1, 4.59, Rotation2d.fromDegrees(-10));
        public static final Pose2d ReachCharingStation = new Pose2d(5.18, MidStationY,
                Rotation2d.fromDegrees(-180));
        public static final Pose2d MidChargeStation = new Pose2d(2.73, MidStationY, Rotation2d.fromDegrees(-180));

    }

    // Interior waypoints, no rotation needed
    public static final class InteriorWaypoint {
        public static final double practiceDelta = PRACTICE ? 0.6 : 0;
        public static final Translation2d HaflwayToPickup = new Translation2d(4.06 - practiceDelta, 4.24);
        public static final Translation2d HaflwayToBlue1 = new Translation2d(BluePickupX - 0.2 - practiceDelta, 4.24);

        // TODO UPDATE
        public static final Translation2d HaflwayToStation = new Translation2d(5.92, 3.70);
        public static final Translation2d HalfwayUpStation = new Translation2d(3, MidStationY);
    }

}