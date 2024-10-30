package com.userjhansen.automap.Maps

import com.acmerobotics.roadrunner.Pose2d
import com.userjhansen.automap.AutoPart
import com.userjhansen.automap.PartType

class OutsideOne : Map {
    override val startPosition = Pose2d(15.0, -60.0, -Math.PI / 2)

    override val specimenPosition = Pose2d(9.0, -32.0, 0.0)
    override val depositPosition = Pose2d(-46.0, -58.0, Math.PI / 16);

    override val intakeParts = arrayOf(
        arrayOf(
            AutoPart(PartType.STRAFE, Pose2d(12.0, -40.0, 0.0)),
            AutoPart(PartType.SPLINE_TO, Pose2d(30.0, -40.0, Math.PI / 4)),
            AutoPart(PartType.STRAFE_TO, Pose2d(40.0, -34.0, Math.PI / 4)),
        ),
        arrayOf(
            AutoPart(PartType.SPLINE_TO, Pose2d(50.0, -30.0, (2 * Math.PI) / 16), (2 * Math.PI) / 16),
        ),
        arrayOf(
            AutoPart(PartType.SPLINE_TO, Pose2d(55.0, -27.0, (1 * Math.PI) / 16), (1 * Math.PI) / 16),
        ),
    )
    override val depositParts = arrayOf(
        AutoPart(PartType.STRAFE, Pose2d(0.0, -50.0, 0.0)),
        AutoPart(PartType.SPLINE_TO, depositPosition, Math.PI),
    )

    override val parkParts = arrayOf(
        AutoPart(PartType.STRAFE, Pose2d(0.0, -50.0, 0.0)),
        AutoPart(PartType.STRAFE, Pose2d(45.0, -30.0, 0.0)),
        AutoPart(PartType.STRAFE, Pose2d(45.0, -10.0, 0.0)),
        AutoPart(PartType.STRAFE_TO, Pose2d(24.0, -10.0, 0.0)),
    )
}