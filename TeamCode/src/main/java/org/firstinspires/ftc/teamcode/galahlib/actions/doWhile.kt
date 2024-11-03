package org.firstinspires.ftc.teamcode.galahlib.actions

import com.acmerobotics.dashboard.telemetry.TelemetryPacket

class doWhile(val doAction: LoggableAction, val whileAction: LoggableAction) : LoggableAction {

    override val name: String
        get() = "DO_${doAction.name}_WHILE_${whileAction.name}"

    override fun run(p: TelemetryPacket): Boolean {
        doAction.run(p)

        return whileAction.run(p)
    }
}