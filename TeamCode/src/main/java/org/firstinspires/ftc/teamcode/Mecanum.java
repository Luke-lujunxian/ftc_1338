package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.Target;

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

    int i = 0;
    protected double lastL1 = 0;//左前
    protected double lastL2 = 0;//左后
    protected double lastR1 = 0;//右前
    protected double lastR2 = 0;//右后
    protected double changeL1 = 0;//左前
    protected double changeL2 = 0;//左后
    protected double changeR1 = 0;//右前
    protected double changeR2 = 0;//右后
    public void getPositionChange(){
        if(i >= 10){
            changeL1 = MotorL1.getCurrentPosition() - lastL1;
            changeL2 = MotorL2.getCurrentPosition() - lastL2;
            changeR1 = MotorR1.getCurrentPosition() - lastR1;
            changeR2 = MotorR2.getCurrentPosition() - lastR2;
            lastL1 = MotorL1.getCurrentPosition();
            lastL2 = MotorL2.getCurrentPosition();
            lastR1 = MotorR1.getCurrentPosition();
            lastR2 = MotorR2.getCurrentPosition();
            i = 0;
        }
        else
            i++;
    }//获取电机转速
    public double varSpeed(){
        double M1,M2,M3,M4;
        M4 = Math.abs(changeR2);
        M2 = Math.abs(changeL2);
        M3 = Math.abs(changeR1);
        M1 = Math.abs(changeL1);
        double aM = (M1 + M2 + M3 + M4) / 4;
        return (Math.pow((M1 - aM),2) + Math.pow((M2 - aM),2) + Math.pow((M3 - aM),2) + Math.pow((M4 - aM),2)) / 3;
    }//获取速度方差

    public double Targetspeed = 0;
    public double AM = 0;
    private void PIDMove(){
        double RF = Math.abs(changeR1);
        double LB = Math.abs(changeL2);
        double RB = Math.abs(changeR2);
        double LF = Math.abs(changeL1);

        double TargetPosition = Math.min(LF,Math.min(Math.min(RF,LB),RB));
        if(TargetPosition == 0){
            setMacanum();
            return;
        }

        if(IfinRange(RF,TargetPosition - 1,TargetPosition + 1) == false){
            double shift1 = RF - TargetPosition;
            double deltaPower1 = gain(shift1);
            AM = gain(shift1);
            R1 = R1 - deltaPower1 * (R1 / Math.abs(R1));
        }
        if(IfinRange(RB,TargetPosition - 1,TargetPosition + 1) == false){
            double shift2 = RB - TargetPosition;
            double deltaPower2 = gain(shift2);
            R2 = R2 - deltaPower2 * (R2 / Math.abs(R2));
        }
        if(IfinRange(LB,TargetPosition - 1,TargetPosition + 1) == false){
            double shift3 = LB - TargetPosition;
            double deltaPower3 = gain(shift3);
            L2 = L2 - deltaPower3 * (L2 / Math.abs(L2));
        }
        if(IfinRange(LF,TargetPosition - 1,TargetPosition + 1) == false){
            double shift4 = LF - TargetPosition;
            double deltaPower4 = gain(shift4);
            L1 = L1 - deltaPower4 * (L1 / Math.abs(L1));
        }
        Targetspeed = TargetPosition;
        setMacanum();
         return ;
    }

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
            R1 = vx;
            R2 = -vx;
        }
        else if (Math.abs(vx) < Math.abs(vy)){
            L1 = vy;
            L2 = vy;
            R1 = -vy;
            R2 = -vy;
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
            R1 *= p;
            R2 *= p;
        }
        PIDMove();
        return;
    }    //手柄控制
    public void Stick2(double vx,double vy){
        if(Math.abs(vx) > Math.abs(vy)){
            L1 = vx;
            L2 = -vx;
            R1 = vx;
            R2 = -vx;
        }
        else if (Math.abs(vx) < Math.abs(vy)){
            L1 = vy;
            L2 = vy;
            R1 = -vy;
            R2 = -vy;
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
            R1 *= p;
            R2 *= p;
        }
        setMacanum();
        return;
    }

    static public boolean IfinRange(double value, double min, double max){
        if(value >= min && value <= max)
            return true;
        else
            return false;

    }
    static public double gain(double shift){
        double gain;
        shift += 1;
        gain = 0.05 * Math.log10(shift);
        return gain;
    }
}