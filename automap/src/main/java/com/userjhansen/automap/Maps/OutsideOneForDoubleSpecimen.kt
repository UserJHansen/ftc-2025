package com.userjhansen.automap.Maps

import com.acmerobotics.roadrunner.Pose2d
import com.userjhansen.automap.AutoPart
import com.userjhansen.automap.PartType


class OutsideOneForDoubleSpecimen : Map {
    override val startPosition = Pose2d(15.0, -62.0, Math.PI / 2)

    override val specimenPosition = Pose2d(2.0, -30.0, Math.PI / 2)
    override val specimenPosition2 = Pose2d(1.0, -29.0, Math.PI / 2)
    override val depositPosition = Pose2d(-46.0, -58.0, Math.PI / 16)
    override val collectPosition = Pose2d(42.0, -60.0, Math.PI / 2)
    override val parkPosition = Pose2d(42.0, -61.0, Math.PI / 2)

    override val highBasketPosition = Pose2d(-65.0, -47.0, Math.toRadians(315.0))
    override val highBasketPosition2 = Pose2d(-65.0, -50.0, Math.toRadians(315.0))
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
//        AutoPart(PartType.STRAFE, Pose2d(0.0, -50.0, 0.0)),
        AutoPart(PartType.STRAFE_TO, specimenPosition2),
    )
//  Then got to specimen collect position, collect specimen, then repeat this previous action


    override val parkParts = arrayOf(
        AutoPart(PartType.STRAFE_TO, Pose2d(42.0, -62.0, Math.PI / 2)),
        AutoPart(PartType.STRAFE_TO, collectPosition)
    )
    override val specimenParts = arrayOf(
//        Specimen Collecting
        AutoPart(PartType.STRAFE_TO, Pose2d(42.0, -60.0, Math.PI / 2)),
    )
}