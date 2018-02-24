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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "MecanumTest2", group = "Concept")
//@Disabled

/*麦轮控制操作：左摇杆控制平移，平移方向与摇杆方向相同
左右trigger控制旋转，左为逆时针右为顺时针，暂不支持同时使用平移与旋转功能
按住b切换为低速形态*/
public class MecanumTestOP2 extends OpMode {
  private ElapsedTime runtime = new ElapsedTime();
  private Mecanum mecanum = new Mecanum();

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");
    mecanum.MotorL1 = hardwareMap.get(DcMotor.class,"motorLF");
    mecanum.MotorL2 = hardwareMap.get(DcMotor.class,"motorLB");
    mecanum.MotorR1 = hardwareMap.get(DcMotor.class,"motorRF");
    mecanum.MotorR2 = hardwareMap.get(DcMotor.class,"motorRB");
    telemetry.addData("MotorDeclare", "Complete");

  }

  @Override
  public void init_loop() {}

  @Override
  public void start() {
    PIDClass.resetEncode(mecanum.MotorL2);
    PIDClass.resetEncode(mecanum.MotorL1);
    PIDClass.resetEncode(mecanum.MotorR2);
    PIDClass.resetEncode(mecanum.MotorR1);
    runtime.reset();
  }

  @Override
  public void loop() {
    telemetry.update();
    double p;
    if(this.gamepad1.b){
      p = 0.2;
    }
    else{
      p = 1;
    }//低速切换
    telemetry.addData("Status", "Run Time: " + runtime.toString());
    if(this.gamepad1.left_stick_x == 0 && this.gamepad1.left_stick_y == 0){
      mecanum.Circle(p * (this.gamepad1.left_trigger - this.gamepad1.right_trigger));//原地旋转
    }
    else{
      mecanum.Stick2(this.gamepad1.left_stick_x * p,this.gamepad1.left_stick_y * p);//手柄平移
    }
    mecanum.getPositionChange();
    telemetry.addData("LF","%d",mecanum.MotorL1.getCurrentPosition());
    telemetry.addData("LB","%d",mecanum.MotorL2.getCurrentPosition());
    telemetry.addData("RF","%d",mecanum.MotorR1.getCurrentPosition());
    telemetry.addData("RB","%d",mecanum.MotorR2.getCurrentPosition());
    telemetry.addData("LF","%.3f",mecanum.L1);
    telemetry.addData("LB","%.3f",mecanum.L2);
    telemetry.addData("RF","%.3f",mecanum.R1);
    telemetry.addData("RB","%.3f",mecanum.R2);
    telemetry.addData("speedLF","%.3f",mecanum.changeL1);
    telemetry.addData("speedLB","%.3f",mecanum.changeL2);
    telemetry.addData("speedRF","%.3f",mecanum.changeR1);
    telemetry.addData("speedRB","%.3f",mecanum.changeR2);
    telemetry.addData("speedvar","%.3f",mecanum.varSpeed());
    telemetry.addData("Targetspeed","%.3f",mecanum.Targetspeed);
    telemetry.addData("positionvar", "%.3f", mecanum.varPosition());
    telemetry.addData("AM","%.3f",mecanum.AM);
  }
}
