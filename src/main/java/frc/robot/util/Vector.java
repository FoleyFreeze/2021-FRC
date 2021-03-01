package frc.robot.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public class Vector{
    public double r;
    public double theta;

    public Vector(double r, double theta){
        this.r = r;
        this.theta = theta;
    }

    public Vector(Vector vec){
        this.r = vec.r;
        this.theta = vec.theta;
    }

    public static Vector fromXY(double x, double y){
        double r = Math.sqrt(x*x + y*y);
        double theta;
        if(x == 0 && y == 0) theta = 0;
        else theta = Math.atan2(y, x);
        return new Vector(r, theta);
    }

    public static Vector fromYX(double x, double y){
        double r = Math.sqrt(x*x + y*y);
        double theta;
        if(x == 0 && y == 0) theta = 0;
        else theta = Math.atan2(x, y);
        return new Vector(r, theta);
    }

    public static Vector fromPose(Pose2d loc){
        return Vector.fromXY(loc.getX(), loc.getY());
    }

    public void scaleNorm(){
        double a = theta % Math.PI;
        double h;

        //make unit "square" into unit circle
        if(a < 0){
            a = (-a) % (Math.PI/2);
            if(a < Math.PI/4) h = 1/Math.cos(a);
            else h = 1/Math.sin(a);
        } else {
            a = a % (Math.PI/2);
            if(a < Math.PI/4) h = 1/Math.cos(a);
            else h = 1/Math.sin(a);
        }
        r = r/h;
    }

    public void threshNorm(){
        //only scaled down if outside of limits
        if(r > 1) r = 1;
        else if(r < -1) r = -1;
    }

    public Vector inverse(){
        r = -r;
        return this;
    }

    public Vector add(Vector v){
        //if a vector is zero, just copy the other one
        if(v.r == 0) return this;
        else if(r == 0){
            r = v.r;
            theta = v.theta;
            return this;
        }
        double tempR = r;

        //otherwise, use law of cosines to calculate the new r and theta
        r = Math.sqrt(r*r + v.r*v.r + 2*r*v.r * Math.cos(v.theta - theta));
        theta = theta + Math.atan2(v.r*Math.sin(v.theta - theta), 
            tempR + v.r*Math.cos(v.theta - theta));

        return this;
    }

    public static Vector add(Vector v1, Vector v2){
        Vector output = new Vector(v1);
        return output.add(v2);
    }

    public Vector subtract(Vector v){
        this.inverse();
        this.add(v);
        return this.inverse();
    }

    public static Vector subtract(Vector v1, Vector v2){
        Vector out = new Vector(v2);
        return out.inverse().add(v1);
    }

    public String toString(){
        return String.format("%.2f, %.0f", r, Math.toDegrees(theta));
    }

    public String toStringXY(){
        return String.format("%.2f, %.2f", getX(), getY());
    }

    public double getX(){
        return r * Math.cos(theta);
    }

    public double getY(){
        return r * Math.sin(theta);
    }
 
    public boolean equals(Vector v){
        return v.r == r && v.theta == theta;
    }
}