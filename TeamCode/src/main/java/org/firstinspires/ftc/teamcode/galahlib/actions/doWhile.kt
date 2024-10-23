package org.firstinspires.ftc.teamcode.galahlib.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action

class doWhile(val doAction: Action, val whileAction: Action) : Action {
    override fun run(p: TelemetryPacket): Boolean {
        doAction.run(p)

        return whileAction.run(p)
    }
}