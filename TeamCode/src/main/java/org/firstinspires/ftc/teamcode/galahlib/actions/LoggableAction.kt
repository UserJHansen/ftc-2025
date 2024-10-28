package org.firstinspires.ftc.teamcode.galahlib.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action

interface LoggableAction : Action {
    val name: String
}

class Loggable(override val name: String, val action: Action) : LoggableAction {
    override fun run(p: TelemetryPacket): Boolean {
        return action.run(p)
    }

}