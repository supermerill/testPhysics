/*
 * Copyright (c) 2009-2012 jMonkeyEngine
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package jme3Double;

import com.jme3.export.*;
import com.jme3.math.AbstractTriangle;
import com.jme3.math.FastMath;

import java.io.IOException;
import java.util.logging.Logger;

/**
 * <code>PlaneD</code> defines a PlaneD where Normal dot (x,y,z) = Constant.
 * This provides methods for calculating a "distance" of a point from this
 * PlaneD. The distance is pseudo due to the fact that it can be negative if the
 * point is on the non-normal side of the PlaneD.
 * 
 * @author Mark Powell
 * @author Joshua Slack
 */
public class PlaneD implements Savable, Cloneable, java.io.Serializable {

    static final long serialVersionUID = 1;

    private static final Logger logger = Logger
            .getLogger(PlaneD.class.getName());

    public static enum Side {
        None,
        Positive,
        Negative
    }

    /** 
     * Vector normal to the PlaneD.
     */
    protected Vector3d normal = new Vector3d();

    /** 
     * Constant of the PlaneD. See formula in class definition.
     */
    protected double constant;

    /**
     * Constructor instantiates a new <code>PlaneD</code> object. This is the
     * default object and contains a normal of (0,0,0) and a constant of 0.
     */
    public PlaneD() {
    }

    /**
     * Constructor instantiates a new <code>PlaneD</code> object. The normal
     * and constant values are set at creation.
     * 
     * @param normal
     *            the normal of the PlaneD.
     * @param constant
     *            the constant of the PlaneD.
     */
    public PlaneD(Vector3d normal, double constant) {
        if (normal == null) {
            throw new IllegalArgumentException("normal cannot be null");
        }

        this.normal.set(normal);
        this.constant = constant;
    }

    /**
     * <code>setNormal</code> sets the normal of the PlaneD.
     * 
     * @param normal
     *            the new normal of the PlaneD.
     */
    public void setNormal(Vector3d normal) {
        if (normal == null) {
            throw new IllegalArgumentException("normal cannot be null");
        }
        this.normal.set(normal);
    }

    /**
     * <code>setNormal</code> sets the normal of the PlaneD.
     *
     */
    public void setNormal(double x, double y, double z) {
        this.normal.set(x,y,z);
    }

    /**
     * <code>getNormal</code> retrieves the normal of the PlaneD.
     * 
     * @return the normal of the PlaneD.
     */
    public Vector3d getNormal() {
        return normal;
    }

    /**
     * <code>setConstant</code> sets the constant value that helps define the
     * PlaneD.
     * 
     * @param constant
     *            the new constant value.
     */
    public void setConstant(double constant) {
        this.constant = constant;
    }

    /**
     * <code>getConstant</code> returns the constant of the PlaneD.
     * 
     * @return the constant of the PlaneD.
     */
    public double getConstant() {
        return constant;
    }

    public Vector3d getClosestPoint(Vector3d point, Vector3d store){
//        double t = constant - normal.dot(point);
//        return store.set(normal).multLocal(t).addLocal(point);
        double t = (constant - normal.dot(point)) / normal.dot(normal);
        return store.set(normal).multLocal(t).addLocal(point);
    }

    public Vector3d getClosestPoint(Vector3d point){
        return getClosestPoint(point, new Vector3d());
    }

    public Vector3d reflect(Vector3d point, Vector3d store){
        if (store == null)
            store = new Vector3d();

        double d = pseudoDistance(point);
        store.set(normal).negateLocal().multLocal(d * 2f);
        store.addLocal(point);
        return store;
    }

    /**
     * <code>pseudoDistance</code> calculates the distance from this PlaneD to
     * a provided point. If the point is on the negative side of the PlaneD the
     * distance returned is negative, otherwise it is positive. If the point is
     * on the PlaneD, it is zero.
     * 
     * @param point
     *            the point to check.
     * @return the signed distance from the PlaneD to a point.
     */
    public double pseudoDistance(Vector3d point) {
        return normal.dot(point) - constant;
    }

    /**
     * <code>whichSide</code> returns the side at which a point lies on the
     * PlaneD. The positive values returned are: NEGATIVE_SIDE, POSITIVE_SIDE and
     * NO_SIDE.
     * 
     * @param point
     *            the point to check.
     * @return the side at which the point lies.
     */
    public Side whichSide(Vector3d point) {
    	double dis = pseudoDistance(point);
        if (dis < 0) {
            return Side.Negative;
        } else if (dis > 0) {
            return Side.Positive;
        } else {
            return Side.None;
        }
    }

    public boolean isOnPlaneD(Vector3d point){
    	double dist = pseudoDistance(point);
        if (dist < FastMath.FLT_EPSILON && dist > -FastMath.FLT_EPSILON)
            return true;
        else
            return false;
    }

//    /**
//     * Initialize this PlaneD using the three points of the given triangle.
//     * 
//     * @param t
//     *            the triangle
//     */
//    public void setPlaneDPoints(AbstractTriangle t) {
//        setPlaneDPoints(t.get1(), t.get2(), t.get3());
//    }

    /**
     * Initialize this PlaneD using a point of origin and a normal.
     *
     * @param origin
     * @param normal
     */
    public void setOriginNormal(Vector3d origin, Vector3d normal){
        this.normal.set(normal);
        this.constant = normal.x * origin.x + normal.y * origin.y + normal.z * origin.z;
    }

    /**
     * Initialize the PlaneD using the given 3 points as coplanar.
     * 
     * @param v1
     *            the first point
     * @param v2
     *            the second point
     * @param v3
     *            the third point
     */
    public void setPlanePoints(Vector3d v1, Vector3d v2, Vector3d v3) {
        normal.set(v2).subtractLocal(v1);
        normal.crossLocal(v3.x - v1.x, v3.y - v1.y, v3.z - v1.z)
                .normalizeLocal();
        constant = normal.dot(v1);
    }

    /**
     * <code>toString</code> returns a string thta represents the string
     * representation of this PlaneD. It represents the normal as a
     * <code>Vector3d</code> object, so the format is the following:
     * com.jme.math.PlaneD [Normal: org.jme.math.Vector3d [X=XX.XXXX, Y=YY.YYYY,
     * Z=ZZ.ZZZZ] - Constant: CC.CCCCC]
     * 
     * @return the string representation of this PlaneD.
     */
    @Override
    public String toString() {
        return getClass().getSimpleName() + " [Normal: " + normal + " - Constant: "
                + constant + "]";
    }

    public void write(JmeExporter e) throws IOException {
        OutputCapsule capsule = e.getCapsule(this);
        capsule.write(normal, "normal", Vector3d.ZERO);
        capsule.write(constant, "constant", 0);
    }

    public void read(JmeImporter e) throws IOException {
        InputCapsule capsule = e.getCapsule(this);
        normal = (Vector3d) capsule.readSavable("normal", Vector3d.ZERO.clone());
        constant = capsule.readDouble("constant", 0);
    }

    @Override
    public PlaneD clone() {
        try {
            PlaneD p = (PlaneD) super.clone();
            p.normal = normal.clone();
            return p;
        } catch (CloneNotSupportedException e) {
            throw new AssertionError();
        }
    }
}
