package org.firstinspires.ftc.teamcode.galahlib.mechanisms

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.hardware.rev.RevBlinkinLedDriver
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import java.util.concurrent.TimeUnit

class LEDStrip(hardwareMap: HardwareMap) {
    val driver: RevBlinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver::class.java, "blinkin")

    fun showPattern(colors: List<RevBlinkinLedDriver.BlinkinPattern>, blinksPerSecond: Double): Action {
        return object : Action {
            val timeout = Deadline((1000 / blinksPerSecond).toLong(), TimeUnit.MILLISECONDS)
            var index = 0
            override fun run(p: TelemetryPacket): Boolean {
                if (timeout.hasExpired()) {
                    if (index++ >= colors.size) index = 0

                    driver.setPattern(colors[index])
                }

                return true
            }
        }
    }
}