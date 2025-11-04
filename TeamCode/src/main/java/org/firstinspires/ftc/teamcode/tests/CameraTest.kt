package org.firstinspires.ftc.teamcode.tests

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.Locale

@TeleOp(name = "Camera Frame Capture")
class CameraTest : LinearOpMode() {
  val RESOLUTION_WIDTH: Int = 800
  val RESOLUTION_HEIGHT: Int = 600

  var lastX: Boolean = false
  var frameCount: Int = 0
  var capReqTime: Long = 0

  override fun runOpMode() {

    val aprilTag =
        AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .setLensIntrinsics(667.154, 667.154, 438.702, 286.414)
            // ... these parameters are fx, fy, cx, cy.
            .build()

    val portal: VisionPortal =
        VisionPortal.Builder()
            .setCamera(hardwareMap.get<WebcamName?>(WebcamName::class.java, "turretCamera"))
            .setCameraResolution(Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .addProcessor(aprilTag)
            .build()

    while (!isStopRequested) {
      val x = gamepad1.x

      if (x && !lastX) {
        portal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", frameCount++))
        capReqTime = System.currentTimeMillis()
      }

      lastX = x

      telemetry.addLine("######## Camera Capture Utility ########")
      telemetry.addLine(
          String.format(Locale.US, " > Resolution: %dx%d", RESOLUTION_WIDTH, RESOLUTION_HEIGHT)
      )
      telemetry.addLine(" > Press X (or Square) to capture a frame")
      telemetry.addData(" > Camera Status", portal.getCameraState())

      if (capReqTime != 0L) {
        telemetry.addLine("\nCaptured Frame!")
      }

      if (capReqTime != 0L && System.currentTimeMillis() - capReqTime > 1000) {
        capReqTime = 0
      }

      telemetry.update()
    }
  }
}
