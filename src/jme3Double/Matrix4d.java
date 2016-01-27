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
import com.jme3.math.FastMath;
import com.jme3.math.Matrix4f;
import com.jme3.util.BufferUtils;
import com.jme3.util.TempVars;
import java.io.IOException;
import java.nio.FloatBuffer;
import java.util.logging.Logger;

/**
 * <code>Matrix4d</code> defines and maintains a 4x4 matrix in row major order.
 * This matrix is intended for use in a translation and rotational capacity. 
 * It provides convenience methods for creating the matrix from a multitude 
 * of sources.
 * 
 * Matrices are stored assuming column vectors on the right, with the translation
 * in the rightmost column. Element numbering is row,column, so m03 is the zeroth
 * row, third column, which is the "x" translation part. This means that the implicit
 * storage order is column major. However, the get() and set() functions on double
 * arrays default to row major order!
 *
 * @author Mark Powell
 * @author Joshua Slack
 */
public final class Matrix4d implements Savable, Cloneable, java.io.Serializable {

    static final long serialVersionUID = 1;

    private static final Logger logger = Logger.getLogger(Matrix4d.class.getName());
    public double m00, m01, m02, m03;
    public double m10, m11, m12, m13;
    public double m20, m21, m22, m23;
    public double m30, m31, m32, m33;
    public static final Matrix4d ZERO = new Matrix4d(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    public static final Matrix4d IDENTITY = new Matrix4d();

    /**
     * Constructor instantiates a new <code>Matrix</code> that is set to the
     * identity matrix.
     *  
     */
    public Matrix4d() {
        loadIdentity();
    }

    /**
     * constructs a matrix with the given values.
     */
    public Matrix4d(double m00, double m01, double m02, double m03,
            double m10, double m11, double m12, double m13,
            double m20, double m21, double m22, double m23,
            double m30, double m31, double m32, double m33) {

        this.m00 = m00;
        this.m01 = m01;
        this.m02 = m02;
        this.m03 = m03;
        this.m10 = m10;
        this.m11 = m11;
        this.m12 = m12;
        this.m13 = m13;
        this.m20 = m20;
        this.m21 = m21;
        this.m22 = m22;
        this.m23 = m23;
        this.m30 = m30;
        this.m31 = m31;
        this.m32 = m32;
        this.m33 = m33;
    }

    /**
     * Create a new Matrix4d, given data in column-major format.
     *
     * @param array
     *		An array of 16 doubles in column-major format (translation in elements 12, 13 and 14).
     */
    public Matrix4d(double[] array) {
        set(array, false);
    }

    /**
     * Constructor instantiates a new <code>Matrix</code> that is set to the
     * provided matrix. This constructor copies a given Matrix. If the provided
     * matrix is null, the constructor sets the matrix to the identity.
     * 
     * @param mat
     *            the matrix to copy.
     */
    public Matrix4d(Matrix4d mat) {
        copy(mat);
    }

    /**
     * Constructor instantiates a new <code>Matrix</code> that is set to the
     * provided matrix. This constructor copies a given Matrix. If the provided
     * matrix is null, the constructor sets the matrix to the identity.
     * 
     * @param mat
     *            the matrix to copy.
     */
    public Matrix4d(Matrix4f mat) {
        copy(mat);
    }

    /**
     * <code>copy</code> transfers the contents of a given matrix to this
     * matrix. If a null matrix is supplied, this matrix is set to the identity
     * matrix.
     * 
     * @param matrix
     *            the matrix to copy.
     */
    public void copy(Matrix4d matrix) {
        if (null == matrix) {
            loadIdentity();
        } else {
            m00 = matrix.m00;
            m01 = matrix.m01;
            m02 = matrix.m02;
            m03 = matrix.m03;
            m10 = matrix.m10;
            m11 = matrix.m11;
            m12 = matrix.m12;
            m13 = matrix.m13;
            m20 = matrix.m20;
            m21 = matrix.m21;
            m22 = matrix.m22;
            m23 = matrix.m23;
            m30 = matrix.m30;
            m31 = matrix.m31;
            m32 = matrix.m32;
            m33 = matrix.m33;
        }
    }

    /**
     * <code>copy</code> transfers the contents of a given matrix to this
     * matrix. If a null matrix is supplied, this matrix is set to the identity
     * matrix.
     * 
     * @param matrix
     *            the matrix to copy.
     */
    public void copy(Matrix4f matrix) {
        if (null == matrix) {
            loadIdentity();
        } else {
            m00 = matrix.m00;
            m01 = matrix.m01;
            m02 = matrix.m02;
            m03 = matrix.m03;
            m10 = matrix.m10;
            m11 = matrix.m11;
            m12 = matrix.m12;
            m13 = matrix.m13;
            m20 = matrix.m20;
            m21 = matrix.m21;
            m22 = matrix.m22;
            m23 = matrix.m23;
            m30 = matrix.m30;
            m31 = matrix.m31;
            m32 = matrix.m32;
            m33 = matrix.m33;
        }
    }

//    public void fromFrame(Vector3d location, Vector3d direction, Vector3d up, Vector3d left) {
//        loadIdentity();
//
//        TempVars vars = TempVars.get();
//
//        Vector3d f = vars.vect1.set(direction);
//        Vector3d s = vars.vect2.set(f).crossLocal(up);
//        Vector3d u = vars.vect3.set(s).crossLocal(f);
////        s.normalizeLocal();
////        u.normalizeLocal();
//
//        m00 = s.x;
//        m01 = s.y;
//        m02 = s.z;
//
//        m10 = u.x;
//        m11 = u.y;
//        m12 = u.z;
//
//        m20 = -f.x;
//        m21 = -f.y;
//        m22 = -f.z;
//
////        m00 = -left.x;
////        m10 = -left.y;
////        m20 = -left.z;
////
////        m01 = up.x;
////        m11 = up.y;
////        m21 = up.z;
////
////        m02 = -direction.x;
////        m12 = -direction.y;
////        m22 = -direction.z;
////
//
//        Matrix4d transMatrix = vars.tempMat4;
//        transMatrix.loadIdentity();
//        transMatrix.m03 = -location.x;
//        transMatrix.m13 = -location.y;
//        transMatrix.m23 = -location.z;
//        this.multLocal(transMatrix);
//
//        vars.release();
//
////        transMatrix.multLocal(this);
//
////        set(transMatrix);
//    }

    /**
     * <code>get</code> retrieves the values of this object into
     * a double array in row-major order.
     * 
     * @param matrix
     *            the matrix to set the values into.
     */
    public void get(double[] matrix) {
        get(matrix, true);
    }

    /**
     * <code>set</code> retrieves the values of this object into
     * a double array.
     * 
     * @param matrix
     *            the matrix to set the values into.
     * @param rowMajor
     *            whether the outgoing data is in row or column major order.
     */
    public void get(double[] matrix, boolean rowMajor) {
        if (matrix.length != 16) {
            throw new IllegalArgumentException(
                    "Array must be of size 16.");
        }

        if (rowMajor) {
            matrix[0] = m00;
            matrix[1] = m01;
            matrix[2] = m02;
            matrix[3] = m03;
            matrix[4] = m10;
            matrix[5] = m11;
            matrix[6] = m12;
            matrix[7] = m13;
            matrix[8] = m20;
            matrix[9] = m21;
            matrix[10] = m22;
            matrix[11] = m23;
            matrix[12] = m30;
            matrix[13] = m31;
            matrix[14] = m32;
            matrix[15] = m33;
        } else {
            matrix[0] = m00;
            matrix[4] = m01;
            matrix[8] = m02;
            matrix[12] = m03;
            matrix[1] = m10;
            matrix[5] = m11;
            matrix[9] = m12;
            matrix[13] = m13;
            matrix[2] = m20;
            matrix[6] = m21;
            matrix[10] = m22;
            matrix[14] = m23;
            matrix[3] = m30;
            matrix[7] = m31;
            matrix[11] = m32;
            matrix[15] = m33;
        }
    }

    /**
     * <code>get</code> retrieves a value from the matrix at the given
     * position. If the position is invalid a <code>JmeException</code> is
     * thrown.
     * 
     * @param i
     *            the row index.
     * @param j
     *            the colum index.
     * @return the value at (i, j).
     */
    @SuppressWarnings("fallthrough")
    public double get(int i, int j) {
        switch (i) {
            case 0:
                switch (j) {
                    case 0:
                        return m00;
                    case 1:
                        return m01;
                    case 2:
                        return m02;
                    case 3:
                        return m03;
                }
            case 1:
                switch (j) {
                    case 0:
                        return m10;
                    case 1:
                        return m11;
                    case 2:
                        return m12;
                    case 3:
                        return m13;
                }
            case 2:
                switch (j) {
                    case 0:
                        return m20;
                    case 1:
                        return m21;
                    case 2:
                        return m22;
                    case 3:
                        return m23;
                }
            case 3:
                switch (j) {
                    case 0:
                        return m30;
                    case 1:
                        return m31;
                    case 2:
                        return m32;
                    case 3:
                        return m33;
                }
        }

        logger.warning("Invalid matrix index.");
        throw new IllegalArgumentException("Invalid indices into matrix.");
    }

    /**
     * <code>getColumn</code> returns one of three columns specified by the
     * parameter. This column is returned as a double array of length 4.
     * 
     * @param i
     *            the column to retrieve. Must be between 0 and 3.
     * @return the column specified by the index.
     */
    public double[] getColumn(int i) {
        return getColumn(i, null);
    }

    /**
     * <code>getColumn</code> returns one of three columns specified by the
     * parameter. This column is returned as a double[4].
     * 
     * @param i
     *            the column to retrieve. Must be between 0 and 3.
     * @param store
     *            the double array to store the result in. if null, a new one
     *            is created.
     * @return the column specified by the index.
     */
    public double[] getColumn(int i, double[] store) {
        if (store == null) {
            store = new double[4];
        }
        switch (i) {
            case 0:
                store[0] = m00;
                store[1] = m10;
                store[2] = m20;
                store[3] = m30;
                break;
            case 1:
                store[0] = m01;
                store[1] = m11;
                store[2] = m21;
                store[3] = m31;
                break;
            case 2:
                store[0] = m02;
                store[1] = m12;
                store[2] = m22;
                store[3] = m32;
                break;
            case 3:
                store[0] = m03;
                store[1] = m13;
                store[2] = m23;
                store[3] = m33;
                break;
            default:
                logger.warning("Invalid column index.");
                throw new IllegalArgumentException("Invalid column index. " + i);
        }
        return store;
    }

    /**
     * 
     * <code>setColumn</code> sets a particular column of this matrix to that
     * represented by the provided vector.
     * 
     * @param i
     *            the column to set.
     * @param column
     *            the data to set.
     */
    public void setColumn(int i, double[] column) {

        if (column == null) {
            logger.warning("Column is null. Ignoring.");
            return;
        }
        switch (i) {
            case 0:
                m00 = column[0];
                m10 = column[1];
                m20 = column[2];
                m30 = column[3];
                break;
            case 1:
                m01 = column[0];
                m11 = column[1];
                m21 = column[2];
                m31 = column[3];
                break;
            case 2:
                m02 = column[0];
                m12 = column[1];
                m22 = column[2];
                m32 = column[3];
                break;
            case 3:
                m03 = column[0];
                m13 = column[1];
                m23 = column[2];
                m33 = column[3];
                break;
            default:
                logger.warning("Invalid column index.");
                throw new IllegalArgumentException("Invalid column index. " + i);
        }
    }

    /**
     * <code>set</code> places a given value into the matrix at the given
     * position. If the position is invalid a <code>JmeException</code> is
     * thrown.
     * 
     * @param i
     *            the row index.
     * @param j
     *            the colum index.
     * @param value
     *            the value for (i, j).
     */
    @SuppressWarnings("fallthrough")
    public void set(int i, int j, double value) {
        switch (i) {
            case 0:
                switch (j) {
                    case 0:
                        m00 = value;
                        return;
                    case 1:
                        m01 = value;
                        return;
                    case 2:
                        m02 = value;
                        return;
                    case 3:
                        m03 = value;
                        return;
                }
            case 1:
                switch (j) {
                    case 0:
                        m10 = value;
                        return;
                    case 1:
                        m11 = value;
                        return;
                    case 2:
                        m12 = value;
                        return;
                    case 3:
                        m13 = value;
                        return;
                }
            case 2:
                switch (j) {
                    case 0:
                        m20 = value;
                        return;
                    case 1:
                        m21 = value;
                        return;
                    case 2:
                        m22 = value;
                        return;
                    case 3:
                        m23 = value;
                        return;
                }
            case 3:
                switch (j) {
                    case 0:
                        m30 = value;
                        return;
                    case 1:
                        m31 = value;
                        return;
                    case 2:
                        m32 = value;
                        return;
                    case 3:
                        m33 = value;
                        return;
                }
        }

        logger.warning("Invalid matrix index.");
        throw new IllegalArgumentException("Invalid indices into matrix.");
    }

    /**
     * <code>set</code> sets the values of this matrix from an array of
     * values.
     * 
     * @param matrix
     *            the matrix to set the value to.
     * @throws JmeException
     *             if the array is not of size 16.
     */
    public void set(double[][] matrix) {
        if (matrix.length != 4 || matrix[0].length != 4) {
            throw new IllegalArgumentException(
                    "Array must be of size 16.");
        }

        m00 = matrix[0][0];
        m01 = matrix[0][1];
        m02 = matrix[0][2];
        m03 = matrix[0][3];
        m10 = matrix[1][0];
        m11 = matrix[1][1];
        m12 = matrix[1][2];
        m13 = matrix[1][3];
        m20 = matrix[2][0];
        m21 = matrix[2][1];
        m22 = matrix[2][2];
        m23 = matrix[2][3];
        m30 = matrix[3][0];
        m31 = matrix[3][1];
        m32 = matrix[3][2];
        m33 = matrix[3][3];
    }
    
    
    /**
     * Sets the values of this matrix
     */
    public void set(double m00, double m01, double m02, double m03,
            double m10, double m11, double m12, double m13,
            double m20, double m21, double m22, double m23,
            double m30, double m31, double m32, double m33) {

        this.m00 = m00;
        this.m01 = m01;
        this.m02 = m02;
        this.m03 = m03;
        this.m10 = m10;
        this.m11 = m11;
        this.m12 = m12;
        this.m13 = m13;
        this.m20 = m20;
        this.m21 = m21;
        this.m22 = m22;
        this.m23 = m23;
        this.m30 = m30;
        this.m31 = m31;
        this.m32 = m32;
        this.m33 = m33;
    }

    /**
     * <code>set</code> sets the values of this matrix from another matrix.
     *
     * @param matrix
     *            the matrix to read the value from.
     */
    public Matrix4d set(Matrix4d matrix) {
        m00 = matrix.m00;
        m01 = matrix.m01;
        m02 = matrix.m02;
        m03 = matrix.m03;
        m10 = matrix.m10;
        m11 = matrix.m11;
        m12 = matrix.m12;
        m13 = matrix.m13;
        m20 = matrix.m20;
        m21 = matrix.m21;
        m22 = matrix.m22;
        m23 = matrix.m23;
        m30 = matrix.m30;
        m31 = matrix.m31;
        m32 = matrix.m32;
        m33 = matrix.m33;
        return this;
    }

    /**
     * <code>set</code> sets the values of this matrix from another matrix.
     *
     * @param matrix
     *            the matrix to read the value from.
     */
    public Matrix4d set(Matrix4f matrix) {
        m00 = matrix.m00;
        m01 = matrix.m01;
        m02 = matrix.m02;
        m03 = matrix.m03;
        m10 = matrix.m10;
        m11 = matrix.m11;
        m12 = matrix.m12;
        m13 = matrix.m13;
        m20 = matrix.m20;
        m21 = matrix.m21;
        m22 = matrix.m22;
        m23 = matrix.m23;
        m30 = matrix.m30;
        m31 = matrix.m31;
        m32 = matrix.m32;
        m33 = matrix.m33;
        return this;
    }

    /**
     * <code>set</code> sets the values of this matrix from an array of
     * values assuming that the data is rowMajor order;
     * 
     * @param matrix
     *            the matrix to set the value to.
     */
    public void set(double[] matrix) {
        set(matrix, true);
    }

    /**
     * <code>set</code> sets the values of this matrix from an array of
     * values;
     * 
     * @param matrix
     *            the matrix to set the value to.
     * @param rowMajor
     *            whether the incoming data is in row or column major order.
     */
    public void set(double[] matrix, boolean rowMajor) {
        if (matrix.length != 16) {
            throw new IllegalArgumentException(
                    "Array must be of size 16.");
        }

        if (rowMajor) {
            m00 = matrix[0];
            m01 = matrix[1];
            m02 = matrix[2];
            m03 = matrix[3];
            m10 = matrix[4];
            m11 = matrix[5];
            m12 = matrix[6];
            m13 = matrix[7];
            m20 = matrix[8];
            m21 = matrix[9];
            m22 = matrix[10];
            m23 = matrix[11];
            m30 = matrix[12];
            m31 = matrix[13];
            m32 = matrix[14];
            m33 = matrix[15];
        } else {
            m00 = matrix[0];
            m01 = matrix[4];
            m02 = matrix[8];
            m03 = matrix[12];
            m10 = matrix[1];
            m11 = matrix[5];
            m12 = matrix[9];
            m13 = matrix[13];
            m20 = matrix[2];
            m21 = matrix[6];
            m22 = matrix[10];
            m23 = matrix[14];
            m30 = matrix[3];
            m31 = matrix[7];
            m32 = matrix[11];
            m33 = matrix[15];
        }
    }

    public Matrix4d transpose() {
        double[] tmp = new double[16];
        get(tmp, true);
        Matrix4d mat = new Matrix4d(tmp);
        return mat;
    }

    /**
     * <code>transpose</code> locally transposes this Matrix.
     * 
     * @return this object for chaining.
     */
    public Matrix4d transposeLocal() {
        double tmp = m01;
        m01 = m10;
        m10 = tmp;

        tmp = m02;
        m02 = m20;
        m20 = tmp;

        tmp = m03;
        m03 = m30;
        m30 = tmp;

        tmp = m12;
        m12 = m21;
        m21 = tmp;

        tmp = m13;
        m13 = m31;
        m31 = tmp;

        tmp = m23;
        m23 = m32;
        m32 = tmp;

        return this;
    }

//    /**
//     * <code>toFloatBuffer</code> returns a FloatBuffer object that contains
//     * the matrix data.
//     * 
//     * @return matrix data as a FloatBuffer.
//     */
//    public FloatBuffer toFloatBuffer() {
//        return toFloatBuffer(false);
//    }

//    /**
//     * <code>toFloatBuffer</code> returns a FloatBuffer object that contains the
//     * matrix data.
//     * 
//     * @param columnMajor
//     *            if true, this buffer should be filled with column major data,
//     *            otherwise it will be filled row major.
//     * @return matrix data as a FloatBuffer. The position is set to 0 for
//     *         convenience.
//     */
//    public FloatBuffer toFloatBuffer(boolean columnMajor) {
//        FloatBuffer fb = BufferUtils.createFloatBuffer(16);
//        fillFloatBuffer(fb, columnMajor);
//        fb.rewind();
//        return fb;
//    }
//
//    /**
//     * <code>fillFloatBuffer</code> fills a FloatBuffer object with
//     * the matrix data.
//     * @param fb the buffer to fill, must be correct size
//     * @return matrix data as a FloatBuffer.
//     */
//    public FloatBuffer fillFloatBuffer(FloatBuffer fb) {
//        return fillFloatBuffer(fb, false);
//    }

//    /**
//     * <code>fillFloatBuffer</code> fills a FloatBuffer object with the matrix
//     * data.
//     * 
//     * @param fb
//     *            the buffer to fill, starting at current position. Must have
//     *            room for 16 more doubles.
//     * @param columnMajor
//     *            if true, this buffer should be filled with column major data,
//     *            otherwise it will be filled row major.
//     * @return matrix data as a FloatBuffer. (position is advanced by 16 and any
//     *         limit set is not changed).
//     */
//    public FloatBuffer fillFloatBuffer(FloatBuffer fb, boolean columnMajor) {
////        if (columnMajor) {
////            fb.put(m00).put(m10).put(m20).put(m30);
////            fb.put(m01).put(m11).put(m21).put(m31);
////            fb.put(m02).put(m12).put(m22).put(m32);
////            fb.put(m03).put(m13).put(m23).put(m33);
////        } else {
////            fb.put(m00).put(m01).put(m02).put(m03);
////            fb.put(m10).put(m11).put(m12).put(m13);
////            fb.put(m20).put(m21).put(m22).put(m23);
////            fb.put(m30).put(m31).put(m32).put(m33);
////        }
//
//        TempVars vars = TempVars.get();
//
//
//        fillFloatArray(vars.matrixWrite, columnMajor);
//        fb.put(vars.matrixWrite, 0, 16);
//
//        vars.release();
//
//        return fb;
//    }

    public void fillFloatArray(double[] f, boolean columnMajor) {
        if (columnMajor) {
            f[ 0] = m00;
            f[ 1] = m10;
            f[ 2] = m20;
            f[ 3] = m30;
            f[ 4] = m01;
            f[ 5] = m11;
            f[ 6] = m21;
            f[ 7] = m31;
            f[ 8] = m02;
            f[ 9] = m12;
            f[10] = m22;
            f[11] = m32;
            f[12] = m03;
            f[13] = m13;
            f[14] = m23;
            f[15] = m33;
        } else {
            f[ 0] = m00;
            f[ 1] = m01;
            f[ 2] = m02;
            f[ 3] = m03;
            f[ 4] = m10;
            f[ 5] = m11;
            f[ 6] = m12;
            f[ 7] = m13;
            f[ 8] = m20;
            f[ 9] = m21;
            f[10] = m22;
            f[11] = m23;
            f[12] = m30;
            f[13] = m31;
            f[14] = m32;
            f[15] = m33;
        }
    }

    /**
     * <code>readFloatBuffer</code> reads value for this matrix from a FloatBuffer.
     * @param fb the buffer to read from, must be correct size
     * @return this data as a FloatBuffer.
     */
    public Matrix4d readFloatBuffer(FloatBuffer fb) {
        return readFloatBuffer(fb, false);
    }

    /**
     * <code>readFloatBuffer</code> reads value for this matrix from a FloatBuffer.
     * @param fb the buffer to read from, must be correct size
     * @param columnMajor if true, this buffer should be filled with column
     * 		major data, otherwise it will be filled row major.
     * @return this data as a FloatBuffer.
     */
    public Matrix4d readFloatBuffer(FloatBuffer fb, boolean columnMajor) {

        if (columnMajor) {
            m00 = fb.get();
            m10 = fb.get();
            m20 = fb.get();
            m30 = fb.get();
            m01 = fb.get();
            m11 = fb.get();
            m21 = fb.get();
            m31 = fb.get();
            m02 = fb.get();
            m12 = fb.get();
            m22 = fb.get();
            m32 = fb.get();
            m03 = fb.get();
            m13 = fb.get();
            m23 = fb.get();
            m33 = fb.get();
        } else {
            m00 = fb.get();
            m01 = fb.get();
            m02 = fb.get();
            m03 = fb.get();
            m10 = fb.get();
            m11 = fb.get();
            m12 = fb.get();
            m13 = fb.get();
            m20 = fb.get();
            m21 = fb.get();
            m22 = fb.get();
            m23 = fb.get();
            m30 = fb.get();
            m31 = fb.get();
            m32 = fb.get();
            m33 = fb.get();
        }
        return this;
    }

    /**
     * <code>loadIdentity</code> sets this matrix to the identity matrix,
     * namely all zeros with ones along the diagonal.
     *  
     */
    public void loadIdentity() {
        m01 = m02 = m03 = 0.0f;
        m10 = m12 = m13 = 0.0f;
        m20 = m21 = m23 = 0.0f;
        m30 = m31 = m32 = 0.0f;
        m00 = m11 = m22 = m33 = 1.0f;
    }

    public void fromFrustum(double near, double far, double left, double right, double top, double bottom, boolean parallel) {
        loadIdentity();
        if (parallel) {
            // scale
            m00 = 2.0f / (right - left);
            //m11 = 2.0f / (bottom - top);
            m11 = 2.0f / (top - bottom);
            m22 = -2.0f / (far - near);
            m33 = 1f;

            // translation
            m03 = -(right + left) / (right - left);
            //m31 = -(bottom + top) / (bottom - top);
            m13 = -(top + bottom) / (top - bottom);
            m23 = -(far + near) / (far - near);
        } else {
            m00 = (2.0f * near) / (right - left);
            m11 = (2.0f * near) / (top - bottom);
            m32 = -1.0f;
            m33 = -0.0f;

            // A
            m02 = (right + left) / (right - left);

            // B 
            m12 = (top + bottom) / (top - bottom);

            // C
            m22 = -(far + near) / (far - near);

            // D
            m23 = -(2.0f * far * near) / (far - near);
        }
    }

    /**
     * <code>fromAngleAxis</code> sets this matrix4f to the values specified
     * by an angle and an axis of rotation.  This method creates an object, so
     * use fromAngleNormalAxis if your axis is already normalized.
     * 
     * @param angle
     *            the angle to rotate (in radians).
     * @param axis
     *            the axis of rotation.
     */
    public void fromAngleAxis(double angle, Vector3d axis) {
        Vector3d normAxis = axis.normalize();
        fromAngleNormalAxis(angle, normAxis);
    }

    /**
     * <code>fromAngleNormalAxis</code> sets this matrix4f to the values
     * specified by an angle and a normalized axis of rotation.
     * 
     * @param angle
     *            the angle to rotate (in radians).
     * @param axis
     *            the axis of rotation (already normalized).
     */
    public void fromAngleNormalAxis(double angle, Vector3d axis) {
        zero();
        m33 = 1;

        double fCos = Math.cos(angle);
        double fSin = Math.sin(angle);
        double fOneMinusCos = ((double) 1.0) - fCos;
        double fX2 = axis.x * axis.x;
        double fY2 = axis.y * axis.y;
        double fZ2 = axis.z * axis.z;
        double fXYM = axis.x * axis.y * fOneMinusCos;
        double fXZM = axis.x * axis.z * fOneMinusCos;
        double fYZM = axis.y * axis.z * fOneMinusCos;
        double fXSin = axis.x * fSin;
        double fYSin = axis.y * fSin;
        double fZSin = axis.z * fSin;

        m00 = fX2 * fOneMinusCos + fCos;
        m01 = fXYM - fZSin;
        m02 = fXZM + fYSin;
        m10 = fXYM + fZSin;
        m11 = fY2 * fOneMinusCos + fCos;
        m12 = fYZM - fXSin;
        m20 = fXZM - fYSin;
        m21 = fYZM + fXSin;
        m22 = fZ2 * fOneMinusCos + fCos;
    }

    /**
     * <code>mult</code> multiplies this matrix by a scalar.
     * 
     * @param scalar
     *            the scalar to multiply this matrix by.
     */
    public void multLocal(double scalar) {
        m00 *= scalar;
        m01 *= scalar;
        m02 *= scalar;
        m03 *= scalar;
        m10 *= scalar;
        m11 *= scalar;
        m12 *= scalar;
        m13 *= scalar;
        m20 *= scalar;
        m21 *= scalar;
        m22 *= scalar;
        m23 *= scalar;
        m30 *= scalar;
        m31 *= scalar;
        m32 *= scalar;
        m33 *= scalar;
    }

    public Matrix4d mult(double scalar) {
        Matrix4d out = new Matrix4d();
        out.set(this);
        out.multLocal(scalar);
        return out;
    }

    public Matrix4d mult(double scalar, Matrix4d store) {
        store.set(this);
        store.multLocal(scalar);
        return store;
    }

    /**
     * <code>mult</code> multiplies this matrix with another matrix. The
     * result matrix will then be returned. This matrix will be on the left hand
     * side, while the parameter matrix will be on the right.
     * 
     * @param in2
     *            the matrix to multiply this matrix by.
     * @return the resultant matrix
     */
    public Matrix4d mult(Matrix4d in2) {
        return mult(in2, null);
    }

    /**
     * <code>mult</code> multiplies this matrix with another matrix. The
     * result matrix will then be returned. This matrix will be on the left hand
     * side, while the parameter matrix will be on the right.
     * 
     * @param in2
     *            the matrix to multiply this matrix by.
     * @param store
     *            where to store the result. It is safe for in2 and store to be
     *            the same object.
     * @return the resultant matrix
     */
    public Matrix4d mult(Matrix4d in2, Matrix4d store) {
        if (store == null) {
            store = new Matrix4d();
        }

        double temp00, temp01, temp02, temp03;
        double temp10, temp11, temp12, temp13;
        double temp20, temp21, temp22, temp23;
        double temp30, temp31, temp32, temp33;

        temp00 = m00 * in2.m00
                + m01 * in2.m10
                + m02 * in2.m20
                + m03 * in2.m30;
        temp01 = m00 * in2.m01
                + m01 * in2.m11
                + m02 * in2.m21
                + m03 * in2.m31;
        temp02 = m00 * in2.m02
                + m01 * in2.m12
                + m02 * in2.m22
                + m03 * in2.m32;
        temp03 = m00 * in2.m03
                + m01 * in2.m13
                + m02 * in2.m23
                + m03 * in2.m33;

        temp10 = m10 * in2.m00
                + m11 * in2.m10
                + m12 * in2.m20
                + m13 * in2.m30;
        temp11 = m10 * in2.m01
                + m11 * in2.m11
                + m12 * in2.m21
                + m13 * in2.m31;
        temp12 = m10 * in2.m02
                + m11 * in2.m12
                + m12 * in2.m22
                + m13 * in2.m32;
        temp13 = m10 * in2.m03
                + m11 * in2.m13
                + m12 * in2.m23
                + m13 * in2.m33;

        temp20 = m20 * in2.m00
                + m21 * in2.m10
                + m22 * in2.m20
                + m23 * in2.m30;
        temp21 = m20 * in2.m01
                + m21 * in2.m11
                + m22 * in2.m21
                + m23 * in2.m31;
        temp22 = m20 * in2.m02
                + m21 * in2.m12
                + m22 * in2.m22
                + m23 * in2.m32;
        temp23 = m20 * in2.m03
                + m21 * in2.m13
                + m22 * in2.m23
                + m23 * in2.m33;

        temp30 = m30 * in2.m00
                + m31 * in2.m10
                + m32 * in2.m20
                + m33 * in2.m30;
        temp31 = m30 * in2.m01
                + m31 * in2.m11
                + m32 * in2.m21
                + m33 * in2.m31;
        temp32 = m30 * in2.m02
                + m31 * in2.m12
                + m32 * in2.m22
                + m33 * in2.m32;
        temp33 = m30 * in2.m03
                + m31 * in2.m13
                + m32 * in2.m23
                + m33 * in2.m33;

        store.m00 = temp00;
        store.m01 = temp01;
        store.m02 = temp02;
        store.m03 = temp03;
        store.m10 = temp10;
        store.m11 = temp11;
        store.m12 = temp12;
        store.m13 = temp13;
        store.m20 = temp20;
        store.m21 = temp21;
        store.m22 = temp22;
        store.m23 = temp23;
        store.m30 = temp30;
        store.m31 = temp31;
        store.m32 = temp32;
        store.m33 = temp33;

        return store;
    }

    /**
     * <code>mult</code> multiplies this matrix with another matrix. The
     * results are stored internally and a handle to this matrix will 
     * then be returned. This matrix will be on the left hand
     * side, while the parameter matrix will be on the right.
     * 
     * @param in2
     *            the matrix to multiply this matrix by.
     * @return the resultant matrix
     */
    public Matrix4d multLocal(Matrix4d in2) {
        return mult(in2, this);
    }

    /**
     * <code>mult</code> multiplies a vector about a rotation matrix. The
     * resulting vector is returned as a new Vector3d.
     * 
     * @param vec
     *            vec to multiply against.
     * @return the rotated vector.
     */
    public Vector3d mult(Vector3d vec) {
        return mult(vec, null);
    }

    /**
     * <code>mult</code> multiplies a vector about a rotation matrix and adds
     * translation. The resulting vector is returned.
     * 
     * @param vec
     *            vec to multiply against.
     * @param store
     *            a vector to store the result in. Created if null is passed.
     * @return the rotated vector.
     */
    public Vector3d mult(Vector3d vec, Vector3d store) {
        if (store == null) {
            store = new Vector3d();
        }

        double vx = vec.x, vy = vec.y, vz = vec.z;
        store.x = m00 * vx + m01 * vy + m02 * vz + m03;
        store.y = m10 * vx + m11 * vy + m12 * vz + m13;
        store.z = m20 * vx + m21 * vy + m22 * vz + m23;

        return store;
    }

//    /**
//     * <code>mult</code> multiplies a <code>Vector4f</code> about a rotation
//     * matrix. The resulting vector is returned as a new <code>Vector4f</code>.
//     *
//     * @param vec
//     *            vec to multiply against.
//     * @return the rotated vector.
//     */
//    public Vector4f mult(Vector4f vec) {
//        return mult(vec, null);
//    }
//
//    /**
//     * <code>mult</code> multiplies a <code>Vector4f</code> about a rotation
//     * matrix. The resulting vector is returned.
//     *
//     * @param vec
//     *            vec to multiply against.
//     * @param store
//     *            a vector to store the result in. Created if null is passed.
//     * @return the rotated vector.
//     */
//    public Vector4f mult(Vector4f vec, Vector4f store) {
//        if (null == vec) {
//            logger.warning("Source vector is null, null result returned.");
//            return null;
//        }
//        if (store == null) {
//            store = new Vector4f();
//        }
//
//        double vx = vec.x, vy = vec.y, vz = vec.z, vw = vec.w;
//        store.x = m00 * vx + m01 * vy + m02 * vz + m03 * vw;
//        store.y = m10 * vx + m11 * vy + m12 * vz + m13 * vw;
//        store.z = m20 * vx + m21 * vy + m22 * vz + m23 * vw;
//        store.w = m30 * vx + m31 * vy + m32 * vz + m33 * vw;
//
//        return store;
//    }
//
//    /**
//     * <code>mult</code> multiplies a vector about a rotation matrix. The
//     * resulting vector is returned.
//     *
//     * @param vec
//     *            vec to multiply against.
//     * 
//     * @return the rotated vector.
//     */
//    public Vector4f multAcross(Vector4f vec) {
//        return multAcross(vec, null);
//    }
//
//    /**
//     * <code>mult</code> multiplies a vector about a rotation matrix. The
//     * resulting vector is returned.
//     *
//     * @param vec
//     *            vec to multiply against.
//     * @param store
//     *            a vector to store the result in.  created if null is passed.
//     * @return the rotated vector.
//     */
//    public Vector4f multAcross(Vector4f vec, Vector4f store) {
//        if (null == vec) {
//            logger.warning("Source vector is null, null result returned.");
//            return null;
//        }
//        if (store == null) {
//            store = new Vector4f();
//        }
//
//        double vx = vec.x, vy = vec.y, vz = vec.z, vw = vec.w;
//        store.x = m00 * vx + m10 * vy + m20 * vz + m30 * vw;
//        store.y = m01 * vx + m11 * vy + m21 * vz + m31 * vw;
//        store.z = m02 * vx + m12 * vy + m22 * vz + m32 * vw;
//        store.w = m03 * vx + m13 * vy + m23 * vz + m33 * vw;
//
//        return store;
//    }

    /**
     * <code>multNormal</code> multiplies a vector about a rotation matrix, but
     * does not add translation. The resulting vector is returned.
     *
     * @param vec
     *            vec to multiply against.
     * @param store
     *            a vector to store the result in. Created if null is passed.
     * @return the rotated vector.
     */
    public Vector3d multNormal(Vector3d vec, Vector3d store) {
        if (store == null) {
            store = new Vector3d();
        }

        double vx = vec.x, vy = vec.y, vz = vec.z;
        store.x = m00 * vx + m01 * vy + m02 * vz;
        store.y = m10 * vx + m11 * vy + m12 * vz;
        store.z = m20 * vx + m21 * vy + m22 * vz;

        return store;
    }

    /**
     * <code>multNormal</code> multiplies a vector about a rotation matrix, but
     * does not add translation. The resulting vector is returned.
     *
     * @param vec
     *            vec to multiply against.
     * @param store
     *            a vector to store the result in. Created if null is passed.
     * @return the rotated vector.
     */
    public Vector3d multNormalAcross(Vector3d vec, Vector3d store) {
        if (store == null) {
            store = new Vector3d();
        }

        double vx = vec.x, vy = vec.y, vz = vec.z;
        store.x = m00 * vx + m10 * vy + m20 * vz;
        store.y = m01 * vx + m11 * vy + m21 * vz;
        store.z = m02 * vx + m12 * vy + m22 * vz;

        return store;
    }

    /**
     * <code>mult</code> multiplies a vector about a rotation matrix and adds
     * translation. The w value is returned as a result of
     * multiplying the last column of the matrix by 1.0
     * 
     * @param vec
     *            vec to multiply against.
     * @param store
     *            a vector to store the result in. 
     * @return the W value
     */
    public double multProj(Vector3d vec, Vector3d store) {
        double vx = vec.x, vy = vec.y, vz = vec.z;
        store.x = m00 * vx + m01 * vy + m02 * vz + m03;
        store.y = m10 * vx + m11 * vy + m12 * vz + m13;
        store.z = m20 * vx + m21 * vy + m22 * vz + m23;
        return m30 * vx + m31 * vy + m32 * vz + m33;
    }

    /**
     * <code>mult</code> multiplies a vector about a rotation matrix. The
     * resulting vector is returned.
     * 
     * @param vec
     *            vec to multiply against.
     * @param store
     *            a vector to store the result in.  created if null is passed.
     * @return the rotated vector.
     */
    public Vector3d multAcross(Vector3d vec, Vector3d store) {
        if (null == vec) {
            logger.warning("Source vector is null, null result returned.");
            return null;
        }
        if (store == null) {
            store = new Vector3d();
        }

        double vx = vec.x, vy = vec.y, vz = vec.z;
        store.x = m00 * vx + m10 * vy + m20 * vz + m30 * 1;
        store.y = m01 * vx + m11 * vy + m21 * vz + m31 * 1;
        store.z = m02 * vx + m12 * vy + m22 * vz + m32 * 1;

        return store;
    }

    /**
     * <code>mult</code> multiplies a quaternion about a matrix. The
     * resulting vector is returned.
     *
     * @param vec
     *            vec to multiply against.
     * @param store
     *            a quaternion to store the result in.  created if null is passed.
     * @return store = this * vec
     */
    public Quaterniond mult(Quaterniond vec, Quaterniond store) {

        if (null == vec) {
            logger.warning("Source vector is null, null result returned.");
            return null;
        }
        if (store == null) {
            store = new Quaterniond();
        }

        double x = m00 * vec.x + m10 * vec.y + m20 * vec.z + m30 * vec.w;
        double y = m01 * vec.x + m11 * vec.y + m21 * vec.z + m31 * vec.w;
        double z = m02 * vec.x + m12 * vec.y + m22 * vec.z + m32 * vec.w;
        double w = m03 * vec.x + m13 * vec.y + m23 * vec.z + m33 * vec.w;
        store.x = x;
        store.y = y;
        store.z = z;
        store.w = w;

        return store;
    }

    /**
     * <code>mult</code> multiplies an array of 4 doubles against this rotation 
     * matrix. The results are stored directly in the array. (vec4f x mat4f)
     * 
     * @param vec4f
     *            double array (size 4) to multiply against the matrix.
     * @return the vec4f for chaining.
     */
    public double[] mult(double[] vec4f) {
        if (null == vec4f || vec4f.length != 4) {
            logger.warning("invalid array given, must be nonnull and length 4");
            return null;
        }

        double x = vec4f[0], y = vec4f[1], z = vec4f[2], w = vec4f[3];

        vec4f[0] = m00 * x + m01 * y + m02 * z + m03 * w;
        vec4f[1] = m10 * x + m11 * y + m12 * z + m13 * w;
        vec4f[2] = m20 * x + m21 * y + m22 * z + m23 * w;
        vec4f[3] = m30 * x + m31 * y + m32 * z + m33 * w;

        return vec4f;
    }

    /**
     * <code>mult</code> multiplies an array of 4 doubles against this rotation 
     * matrix. The results are stored directly in the array. (vec4f x mat4f)
     * 
     * @param vec4f
     *            double array (size 4) to multiply against the matrix.
     * @return the vec4f for chaining.
     */
    public double[] multAcross(double[] vec4f) {
        if (null == vec4f || vec4f.length != 4) {
            logger.warning("invalid array given, must be nonnull and length 4");
            return null;
        }

        double x = vec4f[0], y = vec4f[1], z = vec4f[2], w = vec4f[3];

        vec4f[0] = m00 * x + m10 * y + m20 * z + m30 * w;
        vec4f[1] = m01 * x + m11 * y + m21 * z + m31 * w;
        vec4f[2] = m02 * x + m12 * y + m22 * z + m32 * w;
        vec4f[3] = m03 * x + m13 * y + m23 * z + m33 * w;

        return vec4f;
    }

    /**
     * Inverts this matrix as a new Matrix4d.
     * 
     * @return The new inverse matrix
     */
    public Matrix4d invert() {
        return invert(null);
    }

    /**
     * Inverts this matrix and stores it in the given store.
     * 
     * @return The store
     */
    public Matrix4d invert(Matrix4d store) {
        if (store == null) {
            store = new Matrix4d();
        }

        double fA0 = m00 * m11 - m01 * m10;
        double fA1 = m00 * m12 - m02 * m10;
        double fA2 = m00 * m13 - m03 * m10;
        double fA3 = m01 * m12 - m02 * m11;
        double fA4 = m01 * m13 - m03 * m11;
        double fA5 = m02 * m13 - m03 * m12;
        double fB0 = m20 * m31 - m21 * m30;
        double fB1 = m20 * m32 - m22 * m30;
        double fB2 = m20 * m33 - m23 * m30;
        double fB3 = m21 * m32 - m22 * m31;
        double fB4 = m21 * m33 - m23 * m31;
        double fB5 = m22 * m33 - m23 * m32;
        double fDet = fA0 * fB5 - fA1 * fB4 + fA2 * fB3 + fA3 * fB2 - fA4 * fB1 + fA5 * fB0;

        if (Math.abs(fDet) <= 0f) {
            throw new ArithmeticException("This matrix cannot be inverted");
        }

        store.m00 = +m11 * fB5 - m12 * fB4 + m13 * fB3;
        store.m10 = -m10 * fB5 + m12 * fB2 - m13 * fB1;
        store.m20 = +m10 * fB4 - m11 * fB2 + m13 * fB0;
        store.m30 = -m10 * fB3 + m11 * fB1 - m12 * fB0;
        store.m01 = -m01 * fB5 + m02 * fB4 - m03 * fB3;
        store.m11 = +m00 * fB5 - m02 * fB2 + m03 * fB1;
        store.m21 = -m00 * fB4 + m01 * fB2 - m03 * fB0;
        store.m31 = +m00 * fB3 - m01 * fB1 + m02 * fB0;
        store.m02 = +m31 * fA5 - m32 * fA4 + m33 * fA3;
        store.m12 = -m30 * fA5 + m32 * fA2 - m33 * fA1;
        store.m22 = +m30 * fA4 - m31 * fA2 + m33 * fA0;
        store.m32 = -m30 * fA3 + m31 * fA1 - m32 * fA0;
        store.m03 = -m21 * fA5 + m22 * fA4 - m23 * fA3;
        store.m13 = +m20 * fA5 - m22 * fA2 + m23 * fA1;
        store.m23 = -m20 * fA4 + m21 * fA2 - m23 * fA0;
        store.m33 = +m20 * fA3 - m21 * fA1 + m22 * fA0;

        double fInvDet = 1.0f / fDet;
        store.multLocal(fInvDet);

        return store;
    }

    /**
     * Inverts this matrix locally.
     * 
     * @return this
     */
    public Matrix4d invertLocal() {

        double fA0 = m00 * m11 - m01 * m10;
        double fA1 = m00 * m12 - m02 * m10;
        double fA2 = m00 * m13 - m03 * m10;
        double fA3 = m01 * m12 - m02 * m11;
        double fA4 = m01 * m13 - m03 * m11;
        double fA5 = m02 * m13 - m03 * m12;
        double fB0 = m20 * m31 - m21 * m30;
        double fB1 = m20 * m32 - m22 * m30;
        double fB2 = m20 * m33 - m23 * m30;
        double fB3 = m21 * m32 - m22 * m31;
        double fB4 = m21 * m33 - m23 * m31;
        double fB5 = m22 * m33 - m23 * m32;
        double fDet = fA0 * fB5 - fA1 * fB4 + fA2 * fB3 + fA3 * fB2 - fA4 * fB1 + fA5 * fB0;

        if (Math.abs(fDet) <= 0f) {
            return zero();
        }

        double f00 = +m11 * fB5 - m12 * fB4 + m13 * fB3;
        double f10 = -m10 * fB5 + m12 * fB2 - m13 * fB1;
        double f20 = +m10 * fB4 - m11 * fB2 + m13 * fB0;
        double f30 = -m10 * fB3 + m11 * fB1 - m12 * fB0;
        double f01 = -m01 * fB5 + m02 * fB4 - m03 * fB3;
        double f11 = +m00 * fB5 - m02 * fB2 + m03 * fB1;
        double f21 = -m00 * fB4 + m01 * fB2 - m03 * fB0;
        double f31 = +m00 * fB3 - m01 * fB1 + m02 * fB0;
        double f02 = +m31 * fA5 - m32 * fA4 + m33 * fA3;
        double f12 = -m30 * fA5 + m32 * fA2 - m33 * fA1;
        double f22 = +m30 * fA4 - m31 * fA2 + m33 * fA0;
        double f32 = -m30 * fA3 + m31 * fA1 - m32 * fA0;
        double f03 = -m21 * fA5 + m22 * fA4 - m23 * fA3;
        double f13 = +m20 * fA5 - m22 * fA2 + m23 * fA1;
        double f23 = -m20 * fA4 + m21 * fA2 - m23 * fA0;
        double f33 = +m20 * fA3 - m21 * fA1 + m22 * fA0;

        m00 = f00;
        m01 = f01;
        m02 = f02;
        m03 = f03;
        m10 = f10;
        m11 = f11;
        m12 = f12;
        m13 = f13;
        m20 = f20;
        m21 = f21;
        m22 = f22;
        m23 = f23;
        m30 = f30;
        m31 = f31;
        m32 = f32;
        m33 = f33;

        double fInvDet = 1.0f / fDet;
        multLocal(fInvDet);

        return this;
    }

    /**
     * Returns a new matrix representing the adjoint of this matrix.
     * 
     * @return The adjoint matrix
     */
    public Matrix4d adjoint() {
        return adjoint(null);
    }

    public void setTransform(Vector3d position, Vector3d scale, Matrix3d rotMat) {
        // Ordering:
        //    1. Scale
        //    2. Rotate
        //    3. Translate

        // Set up final matrix with scale, rotation and translation
        m00 = scale.x * rotMat.m00;
        m01 = scale.y * rotMat.m01;
        m02 = scale.z * rotMat.m02;
        m03 = position.x;
        m10 = scale.x * rotMat.m10;
        m11 = scale.y * rotMat.m11;
        m12 = scale.z * rotMat.m12;
        m13 = position.y;
        m20 = scale.x * rotMat.m20;
        m21 = scale.y * rotMat.m21;
        m22 = scale.z * rotMat.m22;
        m23 = position.z;

        // No projection term
        m30 = 0;
        m31 = 0;
        m32 = 0;
        m33 = 1;
    }

    /**
     * Places the adjoint of this matrix in store (creates store if null.)
     * 
     * @param store
     *            The matrix to store the result in.  If null, a new matrix is created.
     * @return store
     */
    public Matrix4d adjoint(Matrix4d store) {
        if (store == null) {
            store = new Matrix4d();
        }

        double fA0 = m00 * m11 - m01 * m10;
        double fA1 = m00 * m12 - m02 * m10;
        double fA2 = m00 * m13 - m03 * m10;
        double fA3 = m01 * m12 - m02 * m11;
        double fA4 = m01 * m13 - m03 * m11;
        double fA5 = m02 * m13 - m03 * m12;
        double fB0 = m20 * m31 - m21 * m30;
        double fB1 = m20 * m32 - m22 * m30;
        double fB2 = m20 * m33 - m23 * m30;
        double fB3 = m21 * m32 - m22 * m31;
        double fB4 = m21 * m33 - m23 * m31;
        double fB5 = m22 * m33 - m23 * m32;

        store.m00 = +m11 * fB5 - m12 * fB4 + m13 * fB3;
        store.m10 = -m10 * fB5 + m12 * fB2 - m13 * fB1;
        store.m20 = +m10 * fB4 - m11 * fB2 + m13 * fB0;
        store.m30 = -m10 * fB3 + m11 * fB1 - m12 * fB0;
        store.m01 = -m01 * fB5 + m02 * fB4 - m03 * fB3;
        store.m11 = +m00 * fB5 - m02 * fB2 + m03 * fB1;
        store.m21 = -m00 * fB4 + m01 * fB2 - m03 * fB0;
        store.m31 = +m00 * fB3 - m01 * fB1 + m02 * fB0;
        store.m02 = +m31 * fA5 - m32 * fA4 + m33 * fA3;
        store.m12 = -m30 * fA5 + m32 * fA2 - m33 * fA1;
        store.m22 = +m30 * fA4 - m31 * fA2 + m33 * fA0;
        store.m32 = -m30 * fA3 + m31 * fA1 - m32 * fA0;
        store.m03 = -m21 * fA5 + m22 * fA4 - m23 * fA3;
        store.m13 = +m20 * fA5 - m22 * fA2 + m23 * fA1;
        store.m23 = -m20 * fA4 + m21 * fA2 - m23 * fA0;
        store.m33 = +m20 * fA3 - m21 * fA1 + m22 * fA0;

        return store;
    }

    /**
     * <code>determinant</code> generates the determinate of this matrix.
     * 
     * @return the determinate
     */
    public double determinant() {
        double fA0 = m00 * m11 - m01 * m10;
        double fA1 = m00 * m12 - m02 * m10;
        double fA2 = m00 * m13 - m03 * m10;
        double fA3 = m01 * m12 - m02 * m11;
        double fA4 = m01 * m13 - m03 * m11;
        double fA5 = m02 * m13 - m03 * m12;
        double fB0 = m20 * m31 - m21 * m30;
        double fB1 = m20 * m32 - m22 * m30;
        double fB2 = m20 * m33 - m23 * m30;
        double fB3 = m21 * m32 - m22 * m31;
        double fB4 = m21 * m33 - m23 * m31;
        double fB5 = m22 * m33 - m23 * m32;
        double fDet = fA0 * fB5 - fA1 * fB4 + fA2 * fB3 + fA3 * fB2 - fA4 * fB1 + fA5 * fB0;
        return fDet;
    }

    /**
     * Sets all of the values in this matrix to zero.
     * 
     * @return this matrix
     */
    public Matrix4d zero() {
        m00 = m01 = m02 = m03 = 0.0f;
        m10 = m11 = m12 = m13 = 0.0f;
        m20 = m21 = m22 = m23 = 0.0f;
        m30 = m31 = m32 = m33 = 0.0f;
        return this;
    }

    public Matrix4d add(Matrix4d mat) {
        Matrix4d result = new Matrix4d();
        result.m00 = this.m00 + mat.m00;
        result.m01 = this.m01 + mat.m01;
        result.m02 = this.m02 + mat.m02;
        result.m03 = this.m03 + mat.m03;
        result.m10 = this.m10 + mat.m10;
        result.m11 = this.m11 + mat.m11;
        result.m12 = this.m12 + mat.m12;
        result.m13 = this.m13 + mat.m13;
        result.m20 = this.m20 + mat.m20;
        result.m21 = this.m21 + mat.m21;
        result.m22 = this.m22 + mat.m22;
        result.m23 = this.m23 + mat.m23;
        result.m30 = this.m30 + mat.m30;
        result.m31 = this.m31 + mat.m31;
        result.m32 = this.m32 + mat.m32;
        result.m33 = this.m33 + mat.m33;
        return result;
    }

    /**
     * <code>add</code> adds the values of a parameter matrix to this matrix.
     * 
     * @param mat
     *            the matrix to add to this.
     */
    public void addLocal(Matrix4d mat) {
        m00 += mat.m00;
        m01 += mat.m01;
        m02 += mat.m02;
        m03 += mat.m03;
        m10 += mat.m10;
        m11 += mat.m11;
        m12 += mat.m12;
        m13 += mat.m13;
        m20 += mat.m20;
        m21 += mat.m21;
        m22 += mat.m22;
        m23 += mat.m23;
        m30 += mat.m30;
        m31 += mat.m31;
        m32 += mat.m32;
        m33 += mat.m33;
    }

    public Vector3d toTranslationVector() {
        return new Vector3d(m03, m13, m23);
    }

    public void toTranslationVector(Vector3d vector) {
        vector.set(m03, m13, m23);
    }

    public Quaterniond toRotationQuat() {
        Quaterniond quat = new Quaterniond();
        quat.fromRotationMatrix(toRotationMatrix());
        return quat;
    }

    public void toRotationQuat(Quaterniond q) {
        q.fromRotationMatrix(toRotationMatrix());
    }

    public Matrix3d toRotationMatrix() {
        return new Matrix3d(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    public void toRotationMatrix(Matrix3d mat) {
        mat.m00 = m00;
        mat.m01 = m01;
        mat.m02 = m02;
        mat.m10 = m10;
        mat.m11 = m11;
        mat.m12 = m12;
        mat.m20 = m20;
        mat.m21 = m21;
        mat.m22 = m22;
	}

	/**
	 * Retreives the scale vector from the matrix.
	 * 
	 * @return the scale vector
	 */
	public Vector3d toScaleVector() {
		Vector3d result = new Vector3d();
		this.toScaleVector(result);
		return result;
	}

	/**
	 * Retreives the scale vector from the matrix and stores it into a given
	 * vector.
	 * 
	 * @param the
	 *            vector where the scale will be stored
	 */
	public void toScaleVector(Vector3d vector) {
		double scaleX = (double) Math.sqrt(m00 * m00 + m10 * m10 + m20 * m20);
		double scaleY = (double) Math.sqrt(m01 * m01 + m11 * m11 + m21 * m21);
		double scaleZ = (double) Math.sqrt(m02 * m02 + m12 * m12 + m22 * m22);
		vector.set(scaleX, scaleY, scaleZ);
	}

    public void setScale(double x, double y, double z) {
        m00 *= x;
        m11 *= y;
        m22 *= z;
    }

    public void setScale(Vector3d scale) {
        m00 *= scale.x;
        m11 *= scale.y;
        m22 *= scale.z;
    }

    /**
     * <code>setTranslation</code> will set the matrix's translation values.
     * 
     * @param translation
     *            the new values for the translation.
     * @throws JmeException
     *             if translation is not size 3.
     */
    public void setTranslation(double[] translation) {
        if (translation.length != 3) {
            throw new IllegalArgumentException(
                    "Translation size must be 3.");
        }
        m03 = translation[0];
        m13 = translation[1];
        m23 = translation[2];
    }

    /**
     * <code>setTranslation</code> will set the matrix's translation values.
     * 
     * @param x
     *            value of the translation on the x axis
     * @param y
     *            value of the translation on the y axis
     * @param z
     *            value of the translation on the z axis
     */
    public void setTranslation(double x, double y, double z) {
        m03 = x;
        m13 = y;
        m23 = z;
    }

    /**
     * <code>setTranslation</code> will set the matrix's translation values.
     *
     * @param translation
     *            the new values for the translation.
     */
    public void setTranslation(Vector3d translation) {
        m03 = translation.x;
        m13 = translation.y;
        m23 = translation.z;
    }

    /**
     * <code>setInverseTranslation</code> will set the matrix's inverse
     * translation values.
     * 
     * @param translation
     *            the new values for the inverse translation.
     * @throws JmeException
     *             if translation is not size 3.
     */
    public void setInverseTranslation(double[] translation) {
        if (translation.length != 3) {
            throw new IllegalArgumentException(
                    "Translation size must be 3.");
        }
        m03 = -translation[0];
        m13 = -translation[1];
        m23 = -translation[2];
    }

    /**
     * <code>angleRotation</code> sets this matrix to that of a rotation about
     * three axes (x, y, z). Where each axis has a specified rotation in
     * degrees. These rotations are expressed in a single <code>Vector3d</code>
     * object.
     * 
     * @param angles
     *            the angles to rotate.
     */
    public void angleRotation(Vector3d angles) {
        double angle;
        double sr, sp, sy, cr, cp, cy;

        angle = (angles.z * FastMath.DEG_TO_RAD);
        sy = Math.sin(angle);
        cy = Math.cos(angle);
        angle = (angles.y * FastMath.DEG_TO_RAD);
        sp = Math.sin(angle);
        cp = Math.cos(angle);
        angle = (angles.x * FastMath.DEG_TO_RAD);
        sr = Math.sin(angle);
        cr = Math.cos(angle);

        // matrix = (Z * Y) * X
        m00 = cp * cy;
        m10 = cp * sy;
        m20 = -sp;
        m01 = sr * sp * cy + cr * -sy;
        m11 = sr * sp * sy + cr * cy;
        m21 = sr * cp;
        m02 = (cr * sp * cy + -sr * -sy);
        m12 = (cr * sp * sy + -sr * cy);
        m22 = cr * cp;
        m03 = 0.0f;
        m13 = 0.0f;
        m23 = 0.0f;
    }

    /**
     * <code>setRotationQuaterniond</code> builds a rotation from a
     * <code>Quaterniond</code>.
     * 
     * @param quat
     *            the quaternion to build the rotation from.
     * @throws NullPointerException
     *             if quat is null.
     */
    public void setRotationQuaterniond(Quaterniond quat) {
        quat.toRotationMatrix(this);
    }

    /**
     * <code>setInverseRotationRadians</code> builds an inverted rotation from
     * Euler angles that are in radians.
     * 
     * @param angles
     *            the Euler angles in radians.
     * @throws JmeException
     *             if angles is not size 3.
     */
    public void setInverseRotationRadians(double[] angles) {
        if (angles.length != 3) {
            throw new IllegalArgumentException(
                    "Angles must be of size 3.");
        }
        double cr = Math.cos(angles[0]);
        double sr = Math.sin(angles[0]);
        double cp = Math.cos(angles[1]);
        double sp = Math.sin(angles[1]);
        double cy = Math.cos(angles[2]);
        double sy = Math.sin(angles[2]);

        m00 = (double) (cp * cy);
        m10 = (double) (cp * sy);
        m20 = (double) (-sp);

        double srsp = sr * sp;
        double crsp = cr * sp;

        m01 = (double) (srsp * cy - cr * sy);
        m11 = (double) (srsp * sy + cr * cy);
        m21 = (double) (sr * cp);

        m02 = (double) (crsp * cy + sr * sy);
        m12 = (double) (crsp * sy - sr * cy);
        m22 = (double) (cr * cp);
    }

    /**
     * <code>setInverseRotationDegrees</code> builds an inverted rotation from
     * Euler angles that are in degrees.
     * 
     * @param angles
     *            the Euler angles in degrees.
     * @throws JmeException
     *             if angles is not size 3.
     */
    public void setInverseRotationDegrees(double[] angles) {
        if (angles.length != 3) {
            throw new IllegalArgumentException(
                    "Angles must be of size 3.");
        }
        double vec[] = new double[3];
        vec[0] = (angles[0] * FastMath.RAD_TO_DEG);
        vec[1] = (angles[1] * FastMath.RAD_TO_DEG);
        vec[2] = (angles[2] * FastMath.RAD_TO_DEG);
        setInverseRotationRadians(vec);
    }

    /**
     * 
     * <code>inverseTranslateVect</code> translates a given Vector3d by the
     * translation part of this matrix.
     * 
     * @param vec
     *            the Vector3d data to be translated.
     * @throws JmeException
     *             if the size of the Vector3d is not 3.
     */
    public void inverseTranslateVect(double[] vec) {
        if (vec.length != 3) {
            throw new IllegalArgumentException(
                    "vec must be of size 3.");
        }

        vec[0] = vec[0] - m03;
        vec[1] = vec[1] - m13;
        vec[2] = vec[2] - m23;
    }

    /**
     * 
     * <code>inverseTranslateVect</code> translates a given Vector3d by the
     * translation part of this matrix.
     * 
     * @param data
     *            the Vector3d to be translated.
     * @throws JmeException
     *             if the size of the Vector3d is not 3.
     */
    public void inverseTranslateVect(Vector3d data) {
        data.x -= m03;
        data.y -= m13;
        data.z -= m23;
    }

    /**
     * 
     * <code>inverseTranslateVect</code> translates a given Vector3d by the
     * translation part of this matrix.
     * 
     * @param data
     *            the Vector3d to be translated.
     * @throws JmeException
     *             if the size of the Vector3d is not 3.
     */
    public void translateVect(Vector3d data) {
        data.x += m03;
        data.y += m13;
        data.z += m23;
    }

    /**
     * 
     * <code>inverseRotateVect</code> rotates a given Vector3d by the rotation
     * part of this matrix.
     * 
     * @param vec
     *            the Vector3d to be rotated.
     */
    public void inverseRotateVect(Vector3d vec) {
        double vx = vec.x, vy = vec.y, vz = vec.z;

        vec.x = vx * m00 + vy * m10 + vz * m20;
        vec.y = vx * m01 + vy * m11 + vz * m21;
        vec.z = vx * m02 + vy * m12 + vz * m22;
    }

    public void rotateVect(Vector3d vec) {
        double vx = vec.x, vy = vec.y, vz = vec.z;

        vec.x = vx * m00 + vy * m01 + vz * m02;
        vec.y = vx * m10 + vy * m11 + vz * m12;
        vec.z = vx * m20 + vy * m21 + vz * m22;
    }

    /**
     * <code>toString</code> returns the string representation of this object.
     * It is in a format of a 4x4 matrix. For example, an identity matrix would
     * be represented by the following string. com.jme.math.Matrix3f <br>[<br>
     * 1.0  0.0  0.0  0.0 <br>
     * 0.0  1.0  0.0  0.0 <br>
     * 0.0  0.0  1.0  0.0 <br>
     * 0.0  0.0  0.0  1.0 <br>]<br>
     * 
     * @return the string representation of this object.
     */
    @Override
    public String toString() {
        StringBuilder result = new StringBuilder("Matrix4d\n[\n");
        result.append(" ");
        result.append(m00);
        result.append("  ");
        result.append(m01);
        result.append("  ");
        result.append(m02);
        result.append("  ");
        result.append(m03);
        result.append(" \n");
        result.append(" ");
        result.append(m10);
        result.append("  ");
        result.append(m11);
        result.append("  ");
        result.append(m12);
        result.append("  ");
        result.append(m13);
        result.append(" \n");
        result.append(" ");
        result.append(m20);
        result.append("  ");
        result.append(m21);
        result.append("  ");
        result.append(m22);
        result.append("  ");
        result.append(m23);
        result.append(" \n");
        result.append(" ");
        result.append(m30);
        result.append("  ");
        result.append(m31);
        result.append("  ");
        result.append(m32);
        result.append("  ");
        result.append(m33);
        result.append(" \n]");
        return result.toString();
    }

    /**
     * 
     * <code>hashCode</code> returns the hash code value as an integer and is
     * supported for the benefit of hashing based collection classes such as
     * Hashtable, HashMap, HashSet etc.
     * 
     * @return the hashcode for this instance of Matrix4d.
     * @see java.lang.Object#hashCode()
     */
    @Override
    public int hashCode() {
        long hash = 37;
        hash = 37 * hash + Double.doubleToLongBits(m00);
        hash = 37 * hash + Double.doubleToLongBits(m01);
        hash = 37 * hash + Double.doubleToLongBits(m02);
        hash = 37 * hash + Double.doubleToLongBits(m03);

        hash = 37 * hash + Double.doubleToLongBits(m10);
        hash = 37 * hash + Double.doubleToLongBits(m11);
        hash = 37 * hash + Double.doubleToLongBits(m12);
        hash = 37 * hash + Double.doubleToLongBits(m13);

        hash = 37 * hash + Double.doubleToLongBits(m20);
        hash = 37 * hash + Double.doubleToLongBits(m21);
        hash = 37 * hash + Double.doubleToLongBits(m22);
        hash = 37 * hash + Double.doubleToLongBits(m23);

        hash = 37 * hash + Double.doubleToLongBits(m30);
        hash = 37 * hash + Double.doubleToLongBits(m31);
        hash = 37 * hash + Double.doubleToLongBits(m32);
        hash = 37 * hash + Double.doubleToLongBits(m33);
        
        return (int)(hash ^ (hash >>> 32));
    }

    /**
     * are these two matrices the same? they are is they both have the same mXX values.
     *
     * @param o
     *            the object to compare for equality
     * @return true if they are equal
     */
    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Matrix4d) || o == null) {
            return false;
        }

        if (this == o) {
            return true;
        }

        Matrix4d comp = (Matrix4d) o;
        if (Double.compare(m00, comp.m00) != 0) {
            return false;
        }
        if (Double.compare(m01, comp.m01) != 0) {
            return false;
        }
        if (Double.compare(m02, comp.m02) != 0) {
            return false;
        }
        if (Double.compare(m03, comp.m03) != 0) {
            return false;
        }

        if (Double.compare(m10, comp.m10) != 0) {
            return false;
        }
        if (Double.compare(m11, comp.m11) != 0) {
            return false;
        }
        if (Double.compare(m12, comp.m12) != 0) {
            return false;
        }
        if (Double.compare(m13, comp.m13) != 0) {
            return false;
        }

        if (Double.compare(m20, comp.m20) != 0) {
            return false;
        }
        if (Double.compare(m21, comp.m21) != 0) {
            return false;
        }
        if (Double.compare(m22, comp.m22) != 0) {
            return false;
        }
        if (Double.compare(m23, comp.m23) != 0) {
            return false;
        }

        if (Double.compare(m30, comp.m30) != 0) {
            return false;
        }
        if (Double.compare(m31, comp.m31) != 0) {
            return false;
        }
        if (Double.compare(m32, comp.m32) != 0) {
            return false;
        }
        if (Double.compare(m33, comp.m33) != 0) {
            return false;
        }

        return true;
    }

    public void write(JmeExporter e) throws IOException {
        OutputCapsule cap = e.getCapsule(this);
        cap.write(m00, "m00", 1);
        cap.write(m01, "m01", 0);
        cap.write(m02, "m02", 0);
        cap.write(m03, "m03", 0);
        cap.write(m10, "m10", 0);
        cap.write(m11, "m11", 1);
        cap.write(m12, "m12", 0);
        cap.write(m13, "m13", 0);
        cap.write(m20, "m20", 0);
        cap.write(m21, "m21", 0);
        cap.write(m22, "m22", 1);
        cap.write(m23, "m23", 0);
        cap.write(m30, "m30", 0);
        cap.write(m31, "m31", 0);
        cap.write(m32, "m32", 0);
        cap.write(m33, "m33", 1);
    }

    public void read(JmeImporter e) throws IOException {
        InputCapsule cap = e.getCapsule(this);
        m00 = cap.readFloat("m00", 1);
        m01 = cap.readFloat("m01", 0);
        m02 = cap.readFloat("m02", 0);
        m03 = cap.readFloat("m03", 0);
        m10 = cap.readFloat("m10", 0);
        m11 = cap.readFloat("m11", 1);
        m12 = cap.readFloat("m12", 0);
        m13 = cap.readFloat("m13", 0);
        m20 = cap.readFloat("m20", 0);
        m21 = cap.readFloat("m21", 0);
        m22 = cap.readFloat("m22", 1);
        m23 = cap.readFloat("m23", 0);
        m30 = cap.readFloat("m30", 0);
        m31 = cap.readFloat("m31", 0);
        m32 = cap.readFloat("m32", 0);
        m33 = cap.readFloat("m33", 1);
    }

    /**
     * @return true if this matrix is identity
     */
    public boolean isIdentity() {
        return (m00 == 1 && m01 == 0 && m02 == 0 && m03 == 0)
                && (m10 == 0 && m11 == 1 && m12 == 0 && m13 == 0)
                && (m20 == 0 && m21 == 0 && m22 == 1 && m23 == 0)
                && (m30 == 0 && m31 == 0 && m32 == 0 && m33 == 1);
    }

    /**
     * Apply a scale to this matrix.
     * 
     * @param scale
     *            the scale to apply
     */
    public void scale(Vector3d scale) {
        m00 *= scale.getX();
        m10 *= scale.getX();
        m20 *= scale.getX();
        m30 *= scale.getX();
        m01 *= scale.getY();
        m11 *= scale.getY();
        m21 *= scale.getY();
        m31 *= scale.getY();
        m02 *= scale.getZ();
        m12 *= scale.getZ();
        m22 *= scale.getZ();
        m32 *= scale.getZ();
    }

    static boolean equalIdentity(Matrix4d mat) {
        if (Math.abs(mat.m00 - 1) > 1e-4) {
            return false;
        }
        if (Math.abs(mat.m11 - 1) > 1e-4) {
            return false;
        }
        if (Math.abs(mat.m22 - 1) > 1e-4) {
            return false;
        }
        if (Math.abs(mat.m33 - 1) > 1e-4) {
            return false;
        }

        if (Math.abs(mat.m01) > 1e-4) {
            return false;
        }
        if (Math.abs(mat.m02) > 1e-4) {
            return false;
        }
        if (Math.abs(mat.m03) > 1e-4) {
            return false;
        }

        if (Math.abs(mat.m10) > 1e-4) {
            return false;
        }
        if (Math.abs(mat.m12) > 1e-4) {
            return false;
        }
        if (Math.abs(mat.m13) > 1e-4) {
            return false;
        }

        if (Math.abs(mat.m20) > 1e-4) {
            return false;
        }
        if (Math.abs(mat.m21) > 1e-4) {
            return false;
        }
        if (Math.abs(mat.m23) > 1e-4) {
            return false;
        }

        if (Math.abs(mat.m30) > 1e-4) {
            return false;
        }
        if (Math.abs(mat.m31) > 1e-4) {
            return false;
        }
        if (Math.abs(mat.m32) > 1e-4) {
            return false;
        }

        return true;
    }

    // XXX: This tests more solid than converting the q to a matrix and multiplying... why?
    public void multLocal(Quaterniond rotation) {
        Vector3d axis = new Vector3d();
        double angle = rotation.toAngleAxis(axis);
        Matrix4d matrix4f = new Matrix4d();
        matrix4f.fromAngleAxis(angle, axis);
        multLocal(matrix4f);
    }

    @Override
    public Matrix4d clone() {
        try {
            return (Matrix4d) super.clone();
        } catch (CloneNotSupportedException e) {
            throw new AssertionError(); // can not happen
        }
    }
}
