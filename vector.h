#pragma once

#include <iostream>
#include <cmath>

class Vector {
    public:
        double x, y, z;
        bool valid;

        Vector() {
            x = 0;
            y = 0;
            z = 0;
            valid = true;
        }

        Vector(bool v_valid) {
            valid = v_valid;
        }

        Vector(double vx, double vy, double vz) {
            x = vx;
            y = vy;
            z = vz;
            valid = true;
        }

        void describe() {
            std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;
        }

        Vector addVector(Vector B) {
            double vx = x + B.x;
            double vy = y + B.y;
            double vz = z + B.z;
            return Vector(vx, vy, vz);
        }

        Vector subtractVector(Vector B) {
            double vx = x - B.x;
            double vy = y - B.y;
            double vz = z - B.z;
            return Vector(vx, vy, vz);
        }

        Vector invert() {
            return Vector(-x, -y, -z);
        }

        Vector scaleByLength(double l) {
            return Vector(x * l, y * l, z * l);
        }

        Vector scaleToLength(double l) {

            if (l == 0) {
                return Vector();
            }

            double mag = magnitude();
            double f = l / mag;
            return Vector(x * f, y * f, z * f);
        }

        double distanceFrom(Vector B) {
            return sqrt(pow(B.x - x, 2) + pow(B.y - y, 2) + pow(B.z - z, 2));
        }

        double dotProduct(Vector B) {
            return x * B.x + y * B.y + z * B.z;
        }

        Vector crossProduct(Vector B) {
            // denoted by A x B
            return Vector(y*B.z - z*B.y, z*B.x - x*B.z, x*B.y - y*B.x);
        }

        double magnitude() {
            // ||v|| denotes the length of a vector
            return sqrt(dotProduct(*this));
        }

        Vector normalise() {
            double mag = magnitude();
            return Vector(x/mag, y/mag, z/mag);
        }

        double angleBetween(Vector B) {
            double a = acos(dotProduct(B) / (magnitude() * B.magnitude()));
            return a;
            // probably need to catch domain error
        }

        Vector bisector(Vector B) {
            return addVector(B).normalise();
        }

        Vector reflectInVector(Vector B) {
            Vector v = normalise();
            Vector normal = B.normalise();
            return v.subtractVector(normal.scaleByLength(2 * v.dotProduct(normal))).normalise();
        }

        Vector reflectInVectorNN(Vector B) {
            // not normalised
            Vector normal = B.normalise();
            return subtractVector(normal.scaleByLength(2 * dotProduct(normal)));
        }

        Vector refractInVector(Vector B, float r_index_a, float r_index_b) {

            Vector v = normalise();
            Vector normal = B.normalise();

            float n = r_index_a / r_index_b;

            double cosI = v.dotProduct(normal);
            if (cosI < -1) {
                cosI = -1;
            }
            else if (cosI > 1) {
                cosI = 1;
            }

            if (cosI < 0) {     // ray is coming from outside object
                cosI = -cosI;
            }
            else {
                n = r_index_b / r_index_a;
                normal = normal.invert();
            }

            double k = 1 - n*n * (1 - cosI*cosI);

            if (k < 0) {
                return Vector(false);
            }

            return v.scaleByLength(n).addVector(normal.scaleByLength(n * cosI - sqrt(k))).normalise();
        }

        Vector matmul(double T[3][3]) {
            double new_x = x*T[0][0] + y*T[0][1] + z*T[0][2];
            double new_y = x*T[1][0] + y*T[1][1] + z*T[1][2];
            double new_z = x*T[2][0] + y*T[2][1] + z*T[2][2];
            return Vector(new_x, new_y, new_z);
        }

        Vector rotateX(double a) {
            double T[3][3] = {
                {1, 0, 0},
                {0, cos(a), -sin(a)},
                {0, sin(a), cos(a)}
            };
            return matmul(T);
        }

        Vector rotateY(double a) {
            double T[3][3] = {
                {cos(a), 0, sin(a)},
                {0, 1, 0},
                {-sin(a), 0, cos(a)}
            };
            return matmul(T);
        }

        Vector rotateIntrinsic(double A, double B, double C) {
            double T[3][3] = {
                {cos(A)*cos(B), cos(A)*sin(B)*sin(C)-sin(A)*cos(C), cos(A)*sin(B)*cos(C)+sin(A)*sin(C)},
                {sin(A)*cos(B), sin(A)*sin(B)*sin(C)+cos(A)*cos(C), sin(A)*sin(B)*cos(C)-cos(A)*sin(C)},
                {-sin(B), cos(B)*sin(C), cos(B)*cos(C)}
            };
            return matmul(T);
        }

        Vector rotateExtrinsic(double A, double B, double C) {
            double T[3][3] = {
                {cos(B)*cos(C), sin(A)*sin(B)*cos(C)-cos(A)*sin(C), cos(A)*sin(B)*cos(C)+sin(A)*sin(C)},
                {cos(B)*sin(C), sin(A)*sin(B)*sin(C)+cos(A)*cos(C), cos(A)*sin(B)*sin(C)-sin(A)*cos(C)},
                {-sin(B), sin(A)*cos(B), cos(A)*cos(B)}
            };
            return matmul(T);
        }

        Vector nullZ() {
            return Vector(x, y, 0);
        }

        double inLine(Vector start, Vector end) {

            // edge case - when end.y - start.y == 0, this does not work

            return (y - start.y) * (end.x - start.x) - (x - start.x) * (end.y - start.y);
            
        }

        Vector pointBetween(Vector B) {
            Vector delta = B.subtractVector(*this).scaleByLength(0.5);
            return addVector(delta);
        }

};