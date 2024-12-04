package com.userjhansen.automap.Maps

import com.acmerobotics.roadrunner.Pose2d
import com.userjhansen.automap.AutoPart
import com.userjhansen.automap.PartType

class OutsideOneSample : Map {
    override val startPosition = Pose2d(-32.5, -62.0, Math.PI / 2)

    override val specimenPosition = Pose2d(2.0, -30.0, Math.PI / 2)
    override val specimenPosition2 = Pose2d(1.0, -29.0, Math.PI / 2)
    override val depositPosition = Pose2d(-46.0, -58.0, Math.PI / 16)
    override val collectPosition = Pose2d(42.0, -60.0, Math.PI / 2)
    override val parkPosition = Pose2d(42.0, -61.0, Math.PI / 2)

    override val highBasketPosition = Pose2d(-65.0, -58.0, Math.PI / 2)
    override val highBasketPosition2 = Pose2d(-65.0, -45.0, Math.PI / 2)
    override val sampleCollectPosition = Pose2d(-49.0, -30.5, Math.PI / 2)

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
        AutoPart(PartType.STRAFE_TO, highBasketPosition, Math.PI),
    )

    override val parkParts = arrayOf(
        AutoPart(PartType.STRAFE_TO, parkPosition),
    )

    override val specimenParts = arrayOf(
      AutoPart(PartType.STRAFE_TO, sampleCollectPosition),
    )

}