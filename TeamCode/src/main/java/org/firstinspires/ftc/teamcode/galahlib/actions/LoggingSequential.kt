package org.firstinspires.ftc.teamcode.galahlib.actions

import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.teamcode.staticData.Logging

class LoggingSequential(val baseName: String, val initialActions: List<LoggableAction>) :
    LoggableAction {
    private var actions = initialActions

    override val name: String
        get() = "[$baseName] ${if (actions.isNotEmpty()) actions.first().name else "FINISHED"}"

    constructor(name: String, vararg actions: LoggableAction) : this(name, actions.asList())

    override tailrec fun run(p: TelemetryPacket): Boolean {
        if (actions.isEmpty()) {
            return false
        }

        return if (actions.first().run(p)) {
            Logging.LOG("${baseName}_STATE", actions.first().name)
            true
        } else {
            if (!name.contains("_FINISHED")) {
                Logging.LOG("${name}_FINISHED")
            }
            actions = actions.drop(1)
            run(p)
        }
    }

    override fun preview(fieldOverlay: Canvas) {
        for (a in initialActions) {
            a.preview(fieldOverlay)
        }
    }
}