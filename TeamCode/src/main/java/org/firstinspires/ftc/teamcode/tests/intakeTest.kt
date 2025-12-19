import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "Intake Test")
class intakeTest : LinearOpMode() {
    override fun runOpMode() {
        val front = hardwareMap.dcMotor["intakeFront"]
        val back = hardwareMap.dcMotor["intakeBack"]
        waitForStart()
        while (opModeIsActive()) {
            front.power = gamepad1.left_stick_y.toDouble()
            back.power = gamepad1.right_stick_y.toDouble()
        }
    }
}
