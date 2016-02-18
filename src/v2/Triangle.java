package v2;

import jme3Double.Vector3d;

public class Triangle {

	public Triangle(Forme f, int i, int j, int k) {
		a = i;
		b = j;
		c = k;
		double ij = f.points.get(i).distance(f.points.get(j));
		double jk = f.points.get(j).distance(f.points.get(k));
		double ki = f.points.get(k).distance(f.points.get(i));
		bbRound = Math.max(Math.max(ij, jk), ki);
		center = f.points.get(i).add(f.points.get(j)).addLocal(f.points.get(k)).divideLocal(3);
	}

	public int a, b, c;
	public double bbRound; // roundbounding box
	public Vector3d center; // in local repere

	public String toString() {
		return a + "," + b + "," + c;
	}

	public static Double EPSILON = 0.0000001d;

	public static boolean collide(Vector3d V0, Vector3d V1, Vector3d V2, Vector3d U0, Vector3d U1, Vector3d U2) {

		// float E1[3],E2[3];
		Vector3d E1 = new Vector3d();
		Vector3d E2 = new Vector3d();
		// float N1[3],N2[3],d1,d2;
		Vector3d N1 = new Vector3d();
		Vector3d N2 = new Vector3d();
		double d1, d2;
		double du0, du1, du2, dv0, dv1, dv2;
		// float D[3];
		Vector3d D = new Vector3d();
		// float isect1[2], isect2[2];
		double isect1_o, isect1_p, isect2_o, isect2_p;
		double du0du1, du0du2, dv0dv1, dv0dv2;
		short index;
		double vp0, vp1, vp2;
		double up0, up1, up2;
		double b, c, max;

		/* compute plane equation of triangle(V0,V1,V2) */
		// SUB(E1,V1,V0);
		V1.subtract(V0, E1);
		// SUB(E2,V2,V0);
		V2.subtract(V0, E2);
		// CROSS(N1,E1,E2);
		E1.cross(E2, N1);
		// d1=-DOT(N1,V0);
		d1 = -N1.dot(V0);
		/* plane equation 1: N1.X+d1=0 */

		/*
		 * put U0,U1,U2 into plane equation 1 to compute signed distances to the
		 * plane
		 */
		// du0=DOT(N1,U0)+d1;
		du0 = N1.dot(U0) + d1;
		// du1=DOT(N1,U1)+d1;
		du1 = N1.dot(U1) + d1;
		// du2=DOT(N1,U2)+d1;
		du2 = N1.dot(U2) + d1;

		/* coplanarity robustness check */
		// #if USE_EPSILON_TEST==TRUE
		// if(Math.abs(du0)<EPSILON) du0=0.0;
		// if(Math.abs(du1)<EPSILON) du1=0.0;
		// if(Math.abs(du2)<EPSILON) du2=0.0;
		// #endif
		du0du1 = du0 * du1;
		du0du2 = du0 * du2;

		if (du0du1 > 0.0f && du0du2 > 0.0f) /*
											 * same sign on all of them + not
											 * equal 0 ?
											 */
			return false; /* no intersection occurs */

		/* compute plane of triangle (U0,U1,U2) */
		// SUB(E1,U1,U0);
		U1.subtract(U0, E1);
		// SUB(E2,U2,U0);
		U2.subtract(U0, E2);
		// CROSS(N2,E1,E2);
		E1.cross(E2, N2);
		// d2=-DOT(N2,U0);
		d2 = -N2.dot(U0);
		/* plane equation 2: N2.X+d2=0 */

		/* put V0,V1,V2 into plane equation 2 */
		// dv0=DOT(N2,V0)+d2;
		dv0 = N2.dot(V0) + d2;
		// dv1=DOT(N2,V1)+d2;
		dv1 = N2.dot(V1) + d2;
		// dv2=DOT(N2,V2)+d2;
		dv2 = N2.dot(V2) + d2;

		// #if USE_EPSILON_TEST==TRUE
		// if(fabs(dv0)<EPSILON) dv0=0.0;
		// if(fabs(dv1)<EPSILON) dv1=0.0;
		// if(fabs(dv2)<EPSILON) dv2=0.0;
		// #endif

		dv0dv1 = dv0 * dv1;
		dv0dv2 = dv0 * dv2;

		if (dv0dv1 > 0.0f && dv0dv2 > 0.0f) /*
											 * same sign on all of them + not
											 * equal 0 ?
											 */
			return false; /* no intersection occurs */

		/* compute direction of intersection line */
		// CROSS(D,N1,N2);
		N1.cross(N2, D);

		/* compute and index to the largest component of D */
		max = Math.abs(D.x);
		index = 0;
		b = Math.abs(D.y);
		c = Math.abs(D.z);
		if (b > max) {
			max = b;
			index = 1;
			vp0 = V0.y;
			vp1 = V1.y;
			vp2 = V2.y;

			up0 = U0.y;
			up1 = U1.y;
			up2 = U2.y;
		} else if (c > max) {
			max = c;
			index = 2;
			vp0 = V0.z;
			vp1 = V1.z;
			vp2 = V2.z;

			up0 = U0.z;
			up1 = U1.z;
			up2 = U2.z;
		} else {
			/* this is the simplified projection onto L */
			vp0 = V0.x;
			vp1 = V1.x;
			vp2 = V2.x;

			up0 = U0.x;
			up1 = U1.x;
			up2 = U2.x;
		}

		/* compute interval for triangle 1 */
		// COMPUTE_INTERVALS(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,isect1[0],isect1[1]);
		{
			double VV0 = vp0;
			double VV1 = vp1;
			double VV2 = vp2;
			double D0 = dv0;
			double D1 = dv1;
			double D2 = dv2;
			double D0D1 = dv0dv1;
			double D0D2 = dv0dv2;
			if (D0D1 > 0.0f) {
				isect1_o = ISECT0(VV2, VV0, VV1, D2, D0, D1);
				isect1_p = ISECT1(VV2, VV0, VV1, D2, D0, D1);
			} else if (D0D2 > 0.0f) {
				isect1_o = ISECT0(VV1, VV0, VV2, D1, D0, D2);
				isect1_p = ISECT1(VV1, VV0, VV2, D1, D0, D2);
			} else if (D1 * D2 > 0.0f || D0 != 0.0f) {
				isect1_o = ISECT0(VV0, VV1, VV2, D0, D1, D2);
				isect1_p = ISECT1(VV0, VV1, VV2, D0, D1, D2);
			} else if (D1 != 0.0f) {
				isect1_o = ISECT0(VV1, VV0, VV2, D1, D0, D2);
				isect1_p = ISECT1(VV1, VV0, VV2, D1, D0, D2);
			} else if (D2 != 0.0f) {
				isect1_o = ISECT0(VV2, VV0, VV1, D2, D0, D1);
				isect1_p = ISECT1(VV2, VV0, VV1, D2, D0, D1);
			} else
				return false; // TODOAFTER: coplanar
		}
		/* compute interval for triangle 2 */
		// COMPUTE_INTERVALS(up0,up1,up2,du0,du1,du2,du0du1,du0du2,isect2[0],isect2[1]);
		{
			double VV0 = up0;
			double VV1 = up1;
			double VV2 = up2;
			double D0 = du0;
			double D1 = du1;
			double D2 = du2;
			double D0D1 = du0du1;
			double D0D2 = du0du2;
			if (D0D1 > 0.0f) {
				isect2_o = ISECT0(VV2, VV0, VV1, D2, D0, D1);
				isect2_p = ISECT1(VV2, VV0, VV1, D2, D0, D1);
			} else if (D0D2 > 0.0f) {
				isect2_o = ISECT0(VV1, VV0, VV2, D1, D0, D2);
				isect2_p = ISECT1(VV1, VV0, VV2, D1, D0, D2);
			} else if (D1 * D2 > 0.0f || D0 != 0.0f) {
				isect2_o = ISECT0(VV0, VV1, VV2, D0, D1, D2);
				isect2_p = ISECT1(VV0, VV1, VV2, D0, D1, D2);
			} else if (D1 != 0.0f) {
				isect2_o = ISECT0(VV1, VV0, VV2, D1, D0, D2);
				isect2_p = ISECT1(VV1, VV0, VV2, D1, D0, D2);
			} else if (D2 != 0.0f) {
				isect2_o = ISECT0(VV2, VV0, VV1, D2, D0, D1);
				isect2_p = ISECT1(VV2, VV0, VV1, D2, D0, D1);
			} else
				return false; // TODOAFTER: coplanar
		}
		// SORT(isect1[0],isect1[1]);
		if (isect1_o > isect1_p) {
			double temp = isect1_o;
			isect1_o = isect1_p;
			isect1_p = temp;
		}
		// SORT(isect2[0],isect2[1]);
		if (isect2_o > isect2_p) {
			double temp = isect2_o;
			isect2_o = isect2_p;
			isect2_p = temp;
		}

		if (isect1_p < isect2_o || isect2_p < isect1_o)
			return false;
		return true;

	}

	public static double ISECT0(double VV0, double VV1, double VV2, double D0, double D1, double D2) {
		return VV0 + (VV1 - VV0) * D0 / (D0 - D1);
	}

	public static double ISECT1(double VV0, double VV1, double VV2, double D0, double D1, double D2) {
		return VV0 + (VV2 - VV0) * D0 / (D0 - D2);
	}

	// public static COMPUTE_INTERVALS(double VV0,double VV1,double VV2,
	// double D0,double D1,double D2,double D0D1,double D0D2,
	// double isect0,double isect1)
	// if(D0D1>0.0f)
	// {
	// /* here we know that D0D2<=0.0 */
	// /* that is D0, D1 are on the same side, D2 on the other or on the plane
	// */
	// // ISECT(VV2,VV0,VV1,D2,D0,D1,isect0,isect1);
	// isect0 = ISECT0(VV2,VV0,VV1,D2,D0,D1);
	// isect1 = ISECT1(VV2,VV0,VV1,D2,D0,D1);
	// }
	// else if(D0D2>0.0f)
	// {
	// /* here we know that d0d1<=0.0 */
	// // ISECT(VV1,VV0,VV2,D1,D0,D2,isect0,isect1);
	// isect0 = ISECT0(VV1,VV0,VV2,D1,D0,D2);
	// isect1 = ISECT1(VV1,VV0,VV2,D1,D0,D2);
	// }
	// else if(D1*D2>0.0f || D0!=0.0f)
	// {
	// /* here we know that d0d1<=0.0 or that D0!=0.0 */
	// // ISECT(VV0,VV1,VV2,D0,D1,D2,isect0,isect1);
	// isect0 = ISECT0(VV0,VV1,VV2,D0,D1,D2);
	// isect1 = ISECT1(VV0,VV1,VV2,D0,D1,D2);
	// }
	// else if(D1!=0.0f)
	// {
	// // ISECT(VV1,VV0,VV2,D1,D0,D2,isect0,isect1);
	// isect0 = ISECT0(VV1,VV0,VV2,D1,D0,D2);
	// isect1 = ISECT1(VV1,VV0,VV2,D1,D0,D2);
	// }
	// else if(D2!=0.0f)
	// {
	// // ISECT(VV2,VV0,VV1,D2,D0,D1,isect0,isect1);
	// isect0 = ISECT0(VV2,VV0,VV1,D2,D0,D1);
	// isect1 = ISECT1(VV2,VV0,VV1,D2,D0,D1);
	// }
	// else
	// {
	// /* triangles are coplanar */
	// //TODOAFTER
	// // return coplanar_tri_tri(N1,V0,V1,V2,U0,U1,U2);
	// return false;
	// }
	// }

	// http://web.archive.org/web/19990203013328/http://www.acm.org/jgt/papers/Moller97/tritri.html
	// int coplanar_tri_tri(Vector3d N,Vector3d V0,Vector3d V1,Vector3d V2,
	// Vector3d U0,Vector3d U1,Vector3d U2)
	// {
	// Vector3d A;
	// short i0,i1;
	// /* first project onto an axis-aligned plane, that maximizes the area */
	// /* of the triangles, compute indices: i0,i1. */
	// A.x=Math.abs(N.x);
	// A.y=Math.abs(N.y);
	// A.z=Math.abs(N.z);
	// if(A.x>A.y)
	// {
	// if(A.x>A.z)
	// {
	// i0=1; /* A.x is greatest */
	// i1=2;
	// }
	// else
	// {
	// i0=0; /* A.z is greatest */
	// i1=1;
	// }
	// }
	// else /* A.x<=A.y */
	// {
	// if(A.z>A.y)
	// {
	// i0=0; /* A.z is greatest */
	// i1=1;
	// }
	// else
	// {
	// i0=0; /* A.y is greatest */
	// i1=2;
	// }
	// }
	//
	// /* test all edges of triangle 1 against the edges of triangle 2 */
	// EDGE_AGAINST_TRI_EDGES(V0,V1,U0,U1,U2);
	// EDGE_AGAINST_TRI_EDGES(V1,V2,U0,U1,U2);
	// EDGE_AGAINST_TRI_EDGES(V2,V0,U0,U1,U2);
	//
	// /* finally, test if tri1 is totally contained in tri2 or vice versa */
	// POINT_IN_TRI(V0,U0,U1,U2);
	// POINT_IN_TRI(U0,V0,V1,V2);
	//
	// return 0;
	// }

}
