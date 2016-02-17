//package v2.collision;
//
//import java.util.HashSet;
//
//import jme3Double.RayD;
//import jme3Double.Vector3d;
//import v2.Forme;
//import v2.Triangle;
//import v2.collision.CollisionPrediction.Precision;
//
//import com.jme3.math.Ray;
//
//public class CollisionRayonTri {
//	Vector3d triA;
//	Vector3d triB;
//	Vector3d triC;
//	Vector3d tempA = new Vector3d();
//	Vector3d tempB = new Vector3d();
//	Vector3d tempC = new Vector3d();
//	Vector3d tempP = new Vector3d();
//	Vector3d tempCalc = new Vector3d();
//	Vector3d rotVelTri = new Vector3d();
//	Vector3d computeVelocity = new Vector3d();
//	Vector3d bestP = new Vector3d();
//	RayD ray = new RayD();
//	HashSet<Integer> badIdxF1 = new HashSet<>();
//	HashSet<Integer> badIdxF2 = new HashSet<>();
//	Forme f1;
//	Forme f2;
//	Forme bestPointeur = null;
//	Forme bestFormeTri = null;
//	double distMin;
//	int bestIdxTri;
//	int bestIdxPoint;
//	CollisionPrediction pred;
//
//	private void initPrediction(CollisionPrediction pred, long currentTime, long dtms) {
//		this.pred = pred;
//		f1 = pred.forme1;
//		f2 = pred.forme2;
//		distMin = 10000000;
//		bestIdxTri = -1;
//		bestIdxPoint = -1;
//
//		badIdxF1.clear();
//		badIdxF1.addAll(f1.joint.getIdx());
//		badIdxF2.clear();
//		badIdxF2.addAll(f2.joint.getIdx());
//		// / END ???
//
//		// create new dir
//		Vector3d vitesseF1 = f2.vitesse.negate().addLocal(f1.vitesse);
//		// ray.setDirection(vitesseF1);
//		System.out.println("--------------- rayCollision ---------- " + vitesseF1);
//		System.out.println("between " + f1 + " and " + f2);
//		
//		checkTriPoint(f1, f2, vitesseF1);
//
//		System.out.println("--------------- rayCollision 2 ---------- " + vitesseF1);
//		System.out.println("between " + f2 + " and " + f1);
//		// change forme
//		// ray.setDirection(vitesseF1.negate());
//		vitesseF1.negateLocal();
//		pointeur = f2;
//		formeTri = f1;
//		for (int idxTri = 0; idxTri < formeTri.triangles.size(); idxTri++) {
//			Triangle tri = formeTri.triangles.get(idxTri);
//			// create the plane for this triangle
//			triA = formeTri.points.get(tri.a);
//			triB = formeTri.points.get(tri.b);
//			triC = formeTri.points.get(tri.c);
//			formeTri.transfoMatrix.mult(triA, tempA);
//			formeTri.transfoMatrix.mult(triB, tempB);
//			formeTri.transfoMatrix.mult(triC, tempC);
//
//			// get the rotational velocity of the triangle
//			tempCalc.set(tempA).addLocal(tempB).addLocal(tempC).divideLocal(3)
//					.subtractLocal(formeTri.position);
//			rotVelTri.set(formeTri.vangulaire).crossLocal(tempCalc).negateLocal();
//
//			for (int idxPoint = 0; idxPoint < pointeur.points.size(); idxPoint++) {
//				if (badIdxF2.contains(idxPoint))
//					continue;
//				Vector3d pointLocalPos = pointeur.points.get(idxPoint);
//				pointeur.transfoMatrix.mult(pointLocalPos, tempP);
//				ray.setOrigin(tempP);
//				// System.out.println("Change point: " +idxPoint+
//				// "@"+tempP+" ("+pointLocalPos+")");
//
//				// add rotationalForce
//				computeVelocity.set(pointeur.vangulaire).crossLocal(tempP.subtract(pointeur.position));
//				computeVelocity.addLocal(vitesseF1).addLocal(rotVelTri);
//				if (computeVelocity.lengthSquared() == 0)
//					continue;
//				ray.setDirection(computeVelocity.normalizeLocal());
//				// System.out.println("2test on tri " +idxTri+
//				// " (p "+idxPoint+") with ray "+ray);
//
//				if (ray.intersectWhere(tempA, tempB, tempC, tempCalc)) {
//					float dist = (tempCalc.distance(tempP));
//					// System.out.println("2find a possible futur intersec on "
//					// + tempCalc + "@"
//					// + (tempCalc.distance(tempP) + " in tri " + idxTri));
//					// System.out.println("from " + ray.getOrigin() +
//					// " (p "+idxPoint+")  with dir " + ray.getDirection());
//					if (dist < distMin) {
//						System.out.println("2find a futur intersec on " + tempCalc + "@"
//								+ (tempCalc.distance(tempP) + " in tri " + idxTri));
//						System.out.println("from " + ray.getOrigin() + " (p " + idxPoint + ") with dir "
//								+ ray.getDirection());
//
//						System.out.println("vitesseF1 = " + vitesseF1 + ", rotVelo= " + computeVelocity
//								+ " rot vector = " + pointeur.vangulaire);
//						System.out.println("1test on tri " + idxTri + " (p " + idxPoint + ") with ray " + ray + " "
//								+ ray.intersectWhere(tempA, tempB, tempC, tempCalc));
//						System.out.println("tri = " + tempA + " " + tempB + " " + tempC);
//						System.out
//								.println("intersection @" + tempCalc + " dist=" + ray.intersects(tempA, tempB, tempC));
//						distMin = dist;
//						bestP.set(tempCalc);
//						bestPointeur = pointeur;
//						bestFormeTri = formeTri;
//						bestIdxTri = idxTri;
//						bestIdxPoint = idxPoint;
//						pred.localTA = triA;
//						pred.localTB = triB;
//						pred.localTC = triC;
//						pred.localP = pointLocalPos;
//						System.out.println("set localP @ " + pointLocalPos);
//						pred.worldTA.set(tempA);
//						pred.worldTB.set(tempB);
//						pred.worldTC.set(tempC);
//						pred.worldP.set(tempP);
//						pred.rayon.set(ray);
//					}
//				}
//			}
//		}
//		System.out.println("--------------- END rayCollision ---------- ");
//		if (bestPointeur != null) {
////			pred.init = true;
////			pred.formePoint = bestPointeur;
////			pred.formeTri = bestFormeTri;
////			pred.pointIdx = bestIdxPoint;
////			pred.triIdx = bestIdxTri;
////			pred.bestP = bestP;
//			// compute time
//			pred.moment = currentTime + (long) (distMin / vitesseF1.length());
//
//			pred.precisionMoment = 
//
//		} // else : no collision possible, check some time later
//			// else, add some time before redo the init? (dangerous!)
//		else {
//
//			// pred.formePoint = bestPointeur;
//			// pred.formeTri = bestFormeTri;
////			pred.pointIdx = -1;
////			pred.triIdx = -1;
////			pred.bestP = null;
////			pred.moment = 0;
//		}
//
//		pred.precisionMoment = Precision.REFINED;
//		// TODO: triangle-triangle collision avec les arretes, sans points.
//
//	}
//
//	private final void checkTriPoint(Forme pointeur, Forme formeTri, Vector3d p2tSpeed) {
//		// check pointe from f1
//		for (int idxTri = 0; idxTri < formeTri.triangles.size(); idxTri++) {
//			Triangle tri = formeTri.triangles.get(idxTri);
//			// create the plane for this triangle
//			triA = formeTri.points.get(tri.a);
//			triB = formeTri.points.get(tri.b);
//			triC = formeTri.points.get(tri.c);
//			formeTri.transfoMatrix.mult(triA, tempA);
//			formeTri.transfoMatrix.mult(triB, tempB);
//			formeTri.transfoMatrix.mult(triC, tempC);
//
//			// get the rotational velocity of the triangle
//			tempCalc.set(tempA).addLocal(tempB).addLocal(tempC).divideLocal(3)
//					.subtractLocal(formeTri.position);
//			rotVelTri.set(formeTri.vangulaire).crossLocal(tempCalc).negateLocal();
//
//			for (int idxPoint = 0; idxPoint < pointeur.points.size(); idxPoint++) {
//				if (badIdxF1.contains(idxPoint))
//					continue;
//				Vector3d pointLocalPos = pointeur.points.get(idxPoint);
//				pointeur.transfoMatrix.mult(pointLocalPos, tempP);
//				// System.out.println("Change point: " +idxPoint+
//				// "@"+tempP+" ("+pointLocalPos+")");
//				ray.setOrigin(tempP);
//				// add rotationalForce
//				computeVelocity.set(pointeur.vangulaire).crossLocal(tempP.subtract(pointeur.position));
//				// System.out.println("vitesseF1 = "+vitesseF1+", rotVelo= "+computeVelocity);
//				computeVelocity.addLocal(p2tSpeed).addLocal(rotVelTri);
//				if (computeVelocity.lengthSquared() == 0)
//					continue;
//				ray.setDirection(computeVelocity.normalizeLocal());
//				// System.out.println("1test on tri " +idxTri+
//				// " (p "+idxPoint+") with ray "+ray+" "+ray.intersectWhere(tempA,
//				// tempB, tempC, tempCalc));
//				// System.out.println("tri = "+tempA+ " "+tempB+" "+ tempC);
//				// System.out.println("intersection @"
//				// +tempCalc+" dist="+ray.intersects(tempA, tempB, tempC));
//
//				if (ray.intersectWhere(tempA, tempB, tempC, tempCalc)) {
//					double dist = (tempCalc.distance(tempP));
//					if (dist < distMin) {
//						System.out.println("1find a futur intersec on " + tempCalc + "@"
//								+ (tempCalc.distance(tempP) + " in tri " + idxTri));
//						System.out.println("from " + ray.getOrigin() + " (p " + idxPoint + ") with dir "
//								+ ray.getDirection());
//
//						System.out.println("vitesseP2T = " + p2tSpeed + ", rotVelo= " + computeVelocity);
//						System.out.println("1test on tri " + idxTri + " (p " + idxPoint + ") with ray " + ray + " "
//								+ ray.intersectWhere(tempA, tempB, tempC, tempCalc));
//						System.out.println("tri = " + tempA + " " + tempB + " " + tempC);
//						System.out
//								.println("intersection @" + tempCalc + " dist=" + ray.intersects(tempA, tempB, tempC));
//						// System.out.println("vlinear = " + vitesseF1);
//						// System.out.println("vAngulTri = " + rotVelTri);
//						// System.out.println("vAngulP = "
//						// +
//						// computeVelocity.set(pointeur.vangulaire).crossLocal(pointLocalPos.normalize()));
//						distMin = dist;
//						bestP.set(tempCalc);
//						bestPointeur = pointeur;
//						bestFormeTri = formeTri;
//						bestIdxTri = idxTri;
//						bestIdxPoint = idxPoint;
////						pred.localTA = triA;
////						pred.localTB = triB;
////						pred.localTC = triC;
//						System.out.println("set localP @ " + pointLocalPos);
////						pred.localP = pointLocalPos;
////						pred.worldTA.set(tempA);
////						pred.worldTB.set(tempB);
////						pred.worldTC.set(tempC);
////						pred.worldP.set(tempP);
////						pred.rayon.set(ray);
//					}
//				}
//			}
//		}
//	}
//
//}
