package frc.team1458.robot

import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.core.BaseRobot
import frc.team1458.lib.drive.TankDrive
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.sensor.NavX
import frc.team1458.lib.sensor.interfaces.AngleSensor
import frc.team1458.lib.odom.EncoderOdom
import frc.team1458.lib.util.LiveDashboard

class  Robot : BaseRobot() {

    val oi: OI = OI()
    val dt: TankDrive = TankDrive(
        leftMaster = SmartMotor.CANtalonSRX(10), // TODO change IDs to real IDs
        rightMaster = SmartMotor.CANtalonSRX(11),
        leftMotors = arrayOf(SmartMotor.CANtalonSRX(12)),
        rightMotors = arrayOf(SmartMotor.CANtalonSRX(13))
    )
    val gyro: AngleSensor = NavX.MXP_I2C().yaw.inverted
    val odom = EncoderOdom(dt.leftEnc, dt.rightEnc, gyro)
    val drivetrainInverted: Boolean = false



    // Runs when the robot is setup (once)
    override fun robotSetup() {
        println("Setup running...")

    }

    // Runs when auto mode is enabled (put actual autonomous code in the while loop)
    override fun runAuto() {
        println("Warning: Sandstorm")

        LiveDashboard.endPath()

        // refresh data or zeroing out

        dt.leftMaster.connectedEncoder.zero()
        dt.rightMaster.connectedEncoder.zero()
        gyro.zero()

        odom.clear()
        odom.setup()
        odom.update()

        val path = PathUtils.generateLinearPath(arrayOf(Pair(0.0, 0.0), Pair(0.0, 0.0), Pair(0.0, 0.0), Pair(0.0, 0.0), Pair(0.0, 0.0), Pair(0.0, 0.0), Pair(0.0, 0.0), Pair(0.0, 0.0),Pair(0.0, 0.0)), 0) // Todo change pair values

        val lookAhead = 0
        val scaling = 0
        val velocity = 0
        val maxV = 0
        val wheelBase = 0
        val pp = PurePursuitFollower(path, lookAhead, scaling, wheelBase, 0.0)

        println("\nEncoder Start Data - left_enc:")

        while (isAutonomous && isEnabled && !pp.finished(Pair(odom.pose.x, odom.pose.y))) {
            odom.update()
            LiveDashboard.putOdom(odom.pose)

            var (l, r) = pp.getControl(Pair(odom.pose.x, odom.pose.y), odom.pose.theta, velocity.toDouble())

            // velocity limit
            if (l > maxV) {
                // l = maxV
                println("Warning: Velocity Limits Enforced!")
            } else if (l < (maxV * -1.0)){
                // l = maxV * -1.0
                println("Warning: Velocity Limits Enforced!")
            }
            if (r > maxV) {
                // r = maxV
                println("Warning: Velocity Limits Enforced!")
            } else if (r < maxV * -1.0){
                // r = maxV * -1.0
                println("Warning: Velocity Limits Enforced!")
            }

            dt.setDriveVelocity(l, r)

            delay(5)
        }
    }

    // Runs when teleoperated mode is enabled (runs once)
    override fun teleopInit() {

    }

    // Runs when the robot receives commands from the diver station (about 50 times a second)
    override fun teleopPeriodic() {
        odom.update()
        LiveDashboard.putOdom(odom.pose)

        dt.arcadeDrive(
            if (drivetrainInverted){
                -0.5 * (oi.throttleAxis.value)
            } else if (oi.slowDownButton.triggered){
                .5 * oi.throttleAxis.value
            } else{
                oi.throttleAxis.value
            } ,
            oi.steerAxis.value
        )

    }

    // Runs when test mode is being ran, not of any concern right now probably
    override fun runTest() {

    }

    // Runs when the robot is disabled (runs once)
    override fun robotDisabled() {

    }

    // Runs while the robot is disabled (many times a second whiled disabled)
    override fun disabledPeriodic() {

    }
}
