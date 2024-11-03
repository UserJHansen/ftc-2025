package org.firstinspires.ftc.teamcode.localization

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Twist2dDual

class FuseLocation(val otosLocalizer: OTOSLocalizer, val threeWheel: ThreeDeadWheelLocalizer) :
    SettableLocalizer {
    override fun setCurrentPose(pose: Pose2d) {
        otosLocalizer.setCurrentPose(pose)
        otosPose = pose
        threeWheelPose = pose
    }

    var otosPose = Pose2d(0.0, 0.0, 0.0)
    var threeWheelPose = Pose2d(0.0, 0.0, 0.0)

    override fun update(): Twist2dDual<Time> {
        val otosTwist = otosLocalizer.update()
        val threeWheelTwist = threeWheel.update()

        otosPose = otosPose.plus(otosTwist.value())
        threeWheelPose = threeWheelPose.plus(threeWheelTwist.value())

        val location = otosTwist.line.plus(threeWheelTwist.line).div(2.0)
        val rotation = otosTwist.angle.plus(threeWheelTwist.angle).div(2.0)

        return Twist2dDual(
            location,
            rotation
        )
    }
}