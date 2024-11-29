package com.userjhansen.automap.Maps

import com.acmerobotics.roadrunner.Pose2d
import com.userjhansen.automap.AutoPart
import com.userjhansen.automap.PartType
import org.w3c.dom.html.HTMLTableCaptionElement

class OutsideOneForDoubleSpecimen : Map {
    override val startPosition = Pose2d(15.0, -62.0, Math.PI / 2)

    override val specimenPosition = Pose2d(2.0, -30.0, 0.0)
    override val depositPosition = Pose2d(-46.0, -58.0, Math.PI / 16)

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
//  Then got to specimen collect position, collect specimen, then repeat this previous action

    override val parkParts = arrayOf(
//        Do this if you want to keep normal Auto but set up for specimen first thing in Teleop
//        Sample 1 Closest to submersible
        AutoPart(PartType.STRAFE_TO, Pose2d(36.0, -39.0, 0.0)),
        AutoPart(PartType.STRAFE_TO, Pose2d(36.0, -10.0, 0.0)),
        AutoPart(PartType.STRAFE_TO, Pose2d(44.0, -10.0, 0.0)),
        AutoPart(PartType.STRAFE_TO, Pose2d(44.0, -60.0, 0.0)),
//        Sample 2 Middle
        AutoPart(PartType.STRAFE_TO, Pose2d(44.0, -10.0, 0.0)),
        AutoPart(PartType.STRAFE_TO, Pose2d(53.0, -10.0, 0.0)),
        AutoPart(PartType.STRAFE_TO, Pose2d(53.0, -60.0, 0.0)),
//        Sample 3 Closest to Field Wall
        AutoPart(PartType.STRAFE_TO, Pose2d(53.0, -10.0, 0.0)),
        AutoPart(PartType.STRAFE_TO, Pose2d(64.0, -10.0, 0.0)),
        AutoPart(PartType.STRAFE_TO, Pose2d(64.0, -60.0, 0.0)),

//        Specimen Collecting
        AutoPart(PartType.STRAFE_TO, Pose2d(40.0, -60.0, 0.0)),
        AutoPart(PartType.TURN, Pose2d(40.0, -60.0, Math.PI / 2)),
        AutoPart(PartType.STRAFE_TO, Pose2d(40.0, -62.0, Math.PI / 2)),

        //Or
        AutoPart(PartType.SPLINE_TO, Pose2d(40.0, -62.0, Math.PI / 2)),

        //Park
//        AutoPart(PartType.STRAFE_TO, Pose2d(59.0, -39.0, 0.0)),
//        AutoPart(PartType.STRAFE_TO, Pose2d(59.0, -60.0, 0.0)),
    )
}