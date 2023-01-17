package org.firstinspires.ftc.teamcode.utils;

public class Vector {
    public double originX, originY, x, y;

    public Vector(double originX, double originY, double x, double y) {
        this.originX = originX;
        this.originY = originY;
        this.x = x;
        this.y = y;
    }

    public Vector(Vector v) {
        this.originX = v.originX;
        this.originY = v.originY;
        this.x = v.x;
        this.y = v.y;
    }

    public double dot(Vector v) {
        return this.x * v.x + this.y * v.y;
    }

    public Vector parallelProjection(Vector v, double x, double y) {
        double dotV = this.dot(v);
        double dotThis = this.dot(this);
        double scale = dotV / dotThis;

        return new Vector(x, y, this.x * scale, this.y * scale);
    }

    public Vector perpendicularProjection(Vector v, double x, double y) {
        Vector parallel = this.parallelProjection(v, x, y);
        return new Vector(x, y, v.x - parallel.x, v.y - parallel.y);
    }

    public double length() {
        return Math.sqrt(x * x + y * y);
    }

    public void scale(double t) {
        this.x *= t;
        this.y *= t;
    }
}