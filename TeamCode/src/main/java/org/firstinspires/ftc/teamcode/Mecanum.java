package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by yuying75618 on 18/2/21.
 */

public class Mecanum{
    protected double L1 = 0;//左前
    protected double L2 = 0;//左后
    protected double R1 = 0;//右前
    protected double R2 = 0;//右后

    protected DcMotor MotorL1,MotorL2,MotorR1,MotorR2;

    private void setMacanum(){
        MotorL1.setPower(L1);
        MotorL2.setPower(L2);
        MotorR1.setPower(R1);
        MotorR2.setPower(R2);
        return;
    } //设置电机速度

    public void Stop(){
        L1 = 0;
        L2 = 0;
        R1 = 0;
        R2 = 0;
        setMacanum();
        return;
    }    //停止
    public void Circle(double w){
        L1 = w;
        L2 = w;
        R1 = w;
        R2 = w;
        setMacanum();
        return;
    }//原地旋转
    public void AutoMove(double v,double aa){
        double a = aa / 180 * Math.PI;
        double vx = v * Math.cos(a);
        double vy = v * Math.sin(a);

        L1 = vy + vx;
        L2 = vy - vx;
        R1 = vy - vx;
        R2 = vy + vx;
        if(L1 == 0 && L2 == 0 ){
            Stop();
            return;
        }

        double p = 1 / Math.max(Math.abs(L1),Math.abs(L2));
        if(p < 1){
            L1 *= p;
            L2 *= p;
            R1 *= -p;
            R2 *= -p;
        }

        this.setMacanum();
        return;
    }//设置速度与角度移动
    public void Stick(double vx,double vy){
        if(Math.abs(vx) > Math.abs(vy)){
            L1 = vx;
            L2 = -vx;
            R1 = -vx;
            R2 = vx;
        }
        else if (Math.abs(vx) < Math.abs(vy)){
            L1 = vy;
            L2 = vy;
            R1 = vy;
            R2 = vy;
        }
        else if(Math.abs(vx) == Math.abs(vy)){
            L1 = 0;
            L2 = 0;
            R1 = 0;
            R2 = 0;
        }

        if(L1 == 0 && L2 == 0 ){
            Stop();
            return;
        }

        double p = 1 / Math.max(Math.abs(L1),Math.abs(L2));
        if(p < 1) {
            L1 *= p;
            L2 *= p;
            R1 *= -p;
            R2 *= -p;
        }
        this.setMacanum();
        return;
    }    //手柄控制

}