package com.userjhansen.automap.Maps

import com.acmerobotics.roadrunner.Pose2d
import com.userjhansen.automap.AutoPart

interface Map {

    val highBasketPosition2: Pose2d
    val parkPosition: Pose2d
    val highBasketPosition: Pose2d
    val sampleCollectPosition: Pose2d
    val specimenPosition2: Pose2d
    val collectPosition: Pose2d
    val startPosition: Pose2d

//    GAME SPECIFIC OPTIMISATION
    val specimenPosition: Pose2d
    val depositPosition: Pose2d

    val intakeParts: Array<Array<AutoPart>>
    val depositParts: Array<AutoPart>
    val parkParts: Array<AutoPart>
    val specimenParts: Array<AutoPart>
}