/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "T2", group = "Concept")
//@Disabled
public class TestOP2 extends OpMode {
    static int VisualResult = 0;

  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor M1,M2,M3,M4;

  public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");
    M1 = hardwareMap.get(DcMotor.class,"motor1");
    M2 = hardwareMap.get(DcMotor.class,"motor2");
    M3 = hardwareMap.get(DcMotor.class,"motor3");
    M4 = hardwareMap.get(DcMotor.class,"motor4");
    telemetry.addData("MotorDeclare", "Complete");

    //Visual

      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

      parameters.vuforiaLicenseKey = "ATFQie7/////AAAAmfcqlMzRHEYtqWyiIdus69o0nEEOtmVEfYJyxYE/NeCYxN/iRG3HDjowiap8OynvREQSp+M5Pi9A+DZ19W8X4bxlWPHiPhlO1U41AaGU3IPIlkPy1C/f2cqSCtHwAU5iuDKQBRjfLsVFOG3423+dsz2pdx4u/oBoAUG2lwQ/vY8nBJY+BsdUrBNvHNYb8n5LtfNpZQfqgCfM/2yJqWQZh0yKEdtKwYlq0Eeasv9p/8LwDOvVnEQX0q2TlZAZ5CktKx1qcxGOIiHchvPNTQ/ZlefsozJEKxAtwNYbCh6Z70T33yrtAmJanb/6vxgpJMP6R57jC4sX1BYceSwBva1WHLiXB+jsJsgbGKRI51O4I2IE";

      parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
      this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

      VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
      VuforiaTrackable relicTemplate = relicTrackables.get(0);
      relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

      telemetry.addData(">", "Press Play to start");
      telemetry.update();

      relicTrackables.activate();

      while (true) {

          RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
          if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

              telemetry.addData("VuMark", "%s visible", vuMark);
              //保存识别结果
              if(RelicRecoveryVuMark.LEFT == vuMark)
                  VisualResult = 0;
              else if(RelicRecoveryVuMark.CENTER == vuMark)
                  VisualResult = 1;
              else if(RelicRecoveryVuMark.RIGHT == vuMark)
                  VisualResult = 2;
              else
                  telemetry.addData("ERROR","请Debug");

              //可能没用的
              OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
              telemetry.addData("Pose", format(pose));

              if (pose != null) {
                  VectorF trans = pose.getTranslation();
                  Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                  double tX = trans.get(0);
                  double tY = trans.get(1);
                  double tZ = trans.get(2);

                  double rX = rot.firstAngle;
                  double rY = rot.secondAngle;
                  double rZ = rot.thirdAngle;
              }
              //结束
              break;
          }
          else {
              telemetry.addData("VuMark", "not visible");
          }

          telemetry.update();
      }
  }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }



  @Override
  public void init_loop() {
  }


  @Override
  public void start() {
    runtime.reset();
  }


  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());
    telemetry.addData("识别结果","%d",VisualResult);
    telemetry.update();
    /*M1.setPower(0.5);
    M2.setPower(-0.5);
    M3.setPower(-0.5);
    M4.setPower(0.5);*/

  }

}
