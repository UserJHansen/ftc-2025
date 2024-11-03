package com.userjhansen.automap.Maps

import com.acmerobotics.roadrunner.Pose2d
import com.userjhansen.automap.AutoPart
import com.userjhansen.automap.PartType

class InsideOne : Map {
    override val startPosition = Pose2d(-35.0, -62.0, -Math.PI / 2)

    override val specimenPosition = Pose2d(-6.0, -34.0, 0.0)
    override val depositPosition = Pose2d(-60.0, -50.0, -(Math.PI / 16) + (Math.PI / 2))

    override val intakeParts = arrayOf(
        arrayOf(
            AutoPart(PartType.SPLINE_TO, Pose2d(-48.0, -40.0, Math.PI / 2.0), Math.PI),
            AutoPart(PartType.STRAFE_TO, Pose2d(-48.0, -35.0, Math.PI / 2.0)),
        ),
        arrayOf(
            AutoPart(PartType.STRAFE_TO, Pose2d(-59.0, -35.0, Math.PI / 2.0)),
        ),
        arrayOf(
            AutoPart(PartType.STRAFE_TO, Pose2d(-54.0, -24.0, -Math.PI)),
            AutoPart(PartType.STRAFE, Pose2d(-57.0, -24.0, -Math.PI)),
        ),
    )

    override val depositParts = arrayOf(
        AutoPart(PartType.STRAFE_TO, depositPosition),
    )

    override val parkParts = arrayOf(
        AutoPart(PartType.STRAFE_TO, Pose2d(-50.0, -13.0, 0.0)),
        AutoPart(PartType.STRAFE_TO, Pose2d(-22.0, -13.0, 0.0)),
    )
}