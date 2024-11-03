package org.firstinspires.ftc.teamcode.galahlib.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.SleepAction

class Timeout(val doAction: LoggableAction, val timeout: Double) : LoggableAction {
    override val name: String
        get() = "TIMEOUT_${doAction.name}_AFTER_${timeout}"

    val sleep = SleepAction(timeout)

    override fun run(p: TelemetryPacket): Boolean {
        return doAction.run(p) && sleep.run(p)
    }
}