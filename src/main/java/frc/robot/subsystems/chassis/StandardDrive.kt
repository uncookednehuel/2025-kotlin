package frc.robot.subsystems.chassis

import com.ctre.phoenix6.swerve.SwerveRequest
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj.XboxController

class StandardDrive (
    private val controller: XboxController
) {
    private val nominalSpeed: Double = 0.5
    private val slowSpeed: Double = 0.25
    private val fastSpeed: Double = 1.0 //Should always be kept at 1

    /* speed multiplier = add the nominal speed multiplier with the scaled difference of the fast speed and the nominal speed,
    and substract the difference of the slow speed and the nominal speed */
    private fun triggerAdjust(slow: Double, fast: Double) = nominalSpeed + (fast * (fastSpeed - nominalSpeed)) - (slow * (slowSpeed - nominalSpeed))

    val chassisSpeeds: ChassisSpeeds
        get() {
            val leftTrigger = MathUtil.applyDeadband(controller.leftTriggerAxis, 0.1)
            val rightTrigger = MathUtil.applyDeadband(controller.rightTriggerAxis, 0.1)

            val speedAdjustment = triggerAdjust(leftTriggerDeadbanded, rightTriggerDeadbanded) * 1 //Use to adjust overall speed (id est training mode)

            val maxSpeed = motionLimits.maxTranslationVelocity * speedAdjustment
            val maxAngularSpeed = motionLimits.maxAngularVelocity * speedAdjustment

            val x = maxSpeed * -controller.leftY
            val y = maxSpeed * -controller.leftX
            val rot = maxAngularSpeed * -controller.rightX
            return ChassisSpeeds(x, y, rot)
        }