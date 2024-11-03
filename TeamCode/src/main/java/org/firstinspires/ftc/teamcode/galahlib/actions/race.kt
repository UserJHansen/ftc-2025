package org.firstinspires.ftc.teamcode.galahlib.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action

class race(val actions: List<Action>) : Action {
    constructor(vararg actions: Action) : this(actions.asList())

    override fun run(p: TelemetryPacket): Boolean {
        return actions.all { it.run(p) }
    }
}