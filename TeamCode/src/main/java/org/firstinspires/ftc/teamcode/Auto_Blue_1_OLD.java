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

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Auto_Blue_1_OLD", group = "Concept")
@Disabled
public class Auto_Blue_1_OLD extends OpMode {
    static int VisualResult = 0;

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor MotorLF, MotorLB, MotorRF, MotorRB;


    //颜色传感器
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    Servo Servo1;
    Servo Servo2;

    public static final String TAG = "Vuforia VuMark Sample";

    OpenGLMatrix lastLocation = null;

    VuforiaLocalizer vuforia;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        //
        MotorLF = hardwareMap.get(DcMotor.class, "motorLF");//左前
        MotorLB = hardwareMap.get(DcMotor.class, "motorLB");//左后
        MotorRF = hardwareMap.get(DcMotor.class, "motorRF");//右前
        MotorRB = hardwareMap.get(DcMotor.class, "motorRB");//右后
        telemetry.addData("MotorInit", "Complete");

        //颜色传感器
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        Servo1 = hardwareMap.get(Servo.class, "ser1");
        Servo2 = hardwareMap.get(Servo.class, "ser2");
        Servo1.setPosition(0.47);
        Servo2.setPosition(0.27);

    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }


    @Override
    public void init_loop() {
      /*
      *Visual
      * 识别壁画
      * 结果会储存在int值VisualResult里
      * 0为左
      * 1为中
      * 2为右
      */
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
                if (RelicRecoveryVuMark.LEFT == vuMark)
                    VisualResult = 0;
                else if (RelicRecoveryVuMark.CENTER == vuMark)
                    VisualResult = 1;
                else if (RelicRecoveryVuMark.RIGHT == vuMark)
                    VisualResult = 2;

                //可能没用的
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
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
            } else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }


    @Override
    public void start() {
        runtime.reset();
        Servo2.setPosition(0.78);

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;           //颜色处理（不用管）


        while (true) {

            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);                                         //别管


            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
            telemetry.addData("Alpha", sensorColor.alpha());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.update();

            int i = 0;
            double lastPos = 0;
            //检测舵机2是否故障
            lastPos = Servo2.getPosition();
            while (Servo2.getPosition() != 0.78) {
                i++;
                if(i==50 && Servo2.getPosition()-lastPos == 0){
                    return;
                }else if(i == 50){
                    i = 0;
                    lastPos = Servo2.getPosition();
                }
            }

            lastPos = Servo1.getPosition();
            if (hsvValues[0] > 250 && hsvValues[0] < 260) {//蓝色
                Servo1.setPosition(0.78);
                //检测舵机1是否故障
                while (Servo1.getPosition() != 0.78) {
                    i++;
                    if(i==50 && Servo1.getPosition()-lastPos == 0){
                        return;
                    }else if(i == 50){
                        i = 0;
                        lastPos = Servo1.getPosition();
                    }
                }
                break;

            }else if(hsvValues[0]>40 && hsvValues[0]<50 ){//红色
                Servo1.setPosition(0);
                //检测舵机1是否故障
                while (Servo1.getPosition() != 0) {
                    i++;
                    if(i==50 && Servo1.getPosition()-lastPos == 0){
                        return;
                    }else if(i == 50){
                        i = 0;
                        lastPos = Servo1.getPosition();
                    }
                }
                break;
            }
        }

        while (Servo1.getPosition() != 0.47){
            Servo1.setPosition(0.47);
        }
        Servo2.setPosition(0.27);


    }


    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("识别结果", "%d", VisualResult);
        telemetry.update();
    /*M1.setPower(0.5);
    M2.setPower(-0.5);
    M3.setPower(-0.5);
    M4.setPower(0.5);*/

    }

}
