package com.userjhansen.automap.Maps

import com.acmerobotics.roadrunner.Pose2d
import com.userjhansen.automap.AutoPart
import com.userjhansen.automap.PartType

class InsideOne : Map {
    override val startPosition = Pose2d(15.0, -62.0, Math.PI / 2)

    override val specimenPosition = Pose2d(2.0, -30.0, Math.PI / 2)
    override val specimenPosition2 = Pose2d(1.0, -29.0, Math.PI / 2)
    override val depositPosition = Pose2d(-46.0, -58.0, Math.PI / 16)
    override val collectPosition = Pose2d(42.0, -60.0, Math.PI / 2)

    override val intakeParts = arrayOf(
        arrayOf(
            AutoPart(PartType.STRAFE, Pose2d(-48.0, -40.0, Math.PI / 2)),
            AutoPart(PartType.STRAFE_TO, Pose2d(-48.0, -35.0, Math.PI / 2)),
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
        AutoPart(PartType.STRAFE_TO, Pose2d(59.0, -39.0, 0.0)),
        AutoPart(PartType.STRAFE_TO, Pose2d(59.0, -60.0, 0.0)),
    )

    override val specimenParts = arrayOf(
//        Specimen Collecting
        AutoPart(PartType.STRAFE_TO, Pose2d(42.0, -60.0, Math.PI / 2)),
    )
}