package frc.team1458.robot

import frc.team1458.lib.actuator.SmartMotor
import frc.team1458.lib.actuator.Solenoid
import frc.team1458.lib.core.BaseRobot
import frc.team1458.lib.drive.TankDrive
import frc.team1458.lib.util.flow.delay
import frc.team1458.lib.sensor.NavX
import frc.team1458.lib.sensor.interfaces.AngleSensor
import frc.team1458.lib.odom.EncoderOdom
import frc.team1458.lib.pathfinding.PathUtils
import frc.team1458.lib.pathfinding.PurePursuitFollower
import frc.team1458.lib.sensor.interfaces.DistanceSensor
import frc.team1458.lib.util.LiveDashboard

class  Robot : BaseRobot() {

    val oi: OI = OI()
    val dt: TankDrive = TankDrive(
        leftMaster = SmartMotor.CANtalonSRX(10), //TODO change IDs to real IDs
        rightMaster = SmartMotor.CANtalonSRX(11),
        leftMotors = arrayOf(SmartMotor.CANtalonSRX(12)),
        rightMotors = arrayOf(SmartMotor.CANtalonSRX(13))
    )
    val gyro: AngleSensor = NavX.MXP_I2C().yaw.inverted
    val odom = EncoderOdom(dt.leftEnc, dt.rightEnc, gyro)
    val drivetrainInverted: Boolean = false
    val intake1 = SmartMotor.CANtalonSRX(17).inverted
    val intake2 = SmartMotor.CANtalonSRX(19)
    val solenoid1 = Solenoid.doubleSolenoid(1,0,0) //TODO change values
    val solenoid2 = Solenoid.doubleSolenoid(2,0,0)
    val solenoid3 = Solenoid.doubleSolenoid(3,0,0)
    val solenoid4 = Solenoid.doubleSolenoid(4,0,0)
    val solenoid5 = Solenoid.doubleSolenoid(5,0,0)
    val solenoid6 = Solenoid.doubleSolenoid(6,0,0)
    val DistanceSensor1 = DistanceSensor.create({0.0},{0.0})
    val DistanceSensor2 = DistanceSensor.create({0.0},{0.0}) //TODO change IDs to real IDs later
    val intakeEnabled = false
    var driveTrainEnabled = true

    var count = 0
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

        val lookAhead = 0 // TODO add values
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

        if (driveTrainEnabled) {
            dt.arcadeDrive(
                if (drivetrainInverted){
                    -0.5 * (oi.throttleAxis.value)
                } else if (oi.slowDownButton.triggered){
                    .5 * oi.throttleAxis.value
                } else{
                    oi.throttleAxis.value
                } ,
                oi.steerAxis.value)
        }
        if(oi.intakeIn.triggered){
            intake1.speed = 1.0 * count
            intake2.speed = 1.0 * count
            count += 1

        }
        else if(oi.intakeOut.triggered) {
            intake1.speed = -1.0 * count
            intake2.speed = -1.0 * count
            count += 1
        }
        else{
            intake1.speed = 0.0
            intake2.speed = 0.0
            count = 0

        }
        if (oi.mastersolenoid.triggered){
            solenoid1.extend()
            solenoid2.extend()
            solenoid3.extend()
            solenoid4.extend()
            println("Solenoids extended.")
            if (DistanceSensor1.distanceMeters == 0.0){ //TODO values later
                println("Solenoids Retracting...")
                driveTrainEnabled = false
                solenoid1.retract()
                solenoid2.retract()
                println("Soldenoids Retracted.")
            }
            driveTrainEnabled = true
            if (DistanceSensor2.distanceMeters == 0.0){ //TODO values later
                println("Solenoids Retracting...")
                driveTrainEnabled = false
                solenoid3.retract()
                solenoid4.retract()
                println("Soldenoids Retracted.")
            }
            if (oi.clawOpen.triggered){ //solenoid 5 is for claw
                println("Solenoid extending...")
                solenoid5.extend()
                println("Solenoid extended...")
            }
            if (oi.clawClose.triggered){
                println("Solenoid retracting")
                solenoid5.retract()
                println("Solenoid retracted")
            }
            if (oi.clawLevel.triggered){ //solenoid 6 is for level
                println("Solenoid extending...")
                solenoid6.extend()
                println("Solenoid extended...")
            }

            driveTrainEnabled = true

            
        }
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
