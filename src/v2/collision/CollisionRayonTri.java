package v2.collision;

import java.util.ArrayList;
import java.util.HashSet;

import jme3Double.RayD;
import jme3Double.Vector3d;
import v2.Forme;
import v2.Triangle;
import v2.collision.CollisionPrediction.Precision;

public class CollisionRayonTri {
	Vector3d triA;
	Vector3d triB;
	Vector3d triC;
	Vector3d tempA = new Vector3d();
	Vector3d tempB = new Vector3d();
	Vector3d tempC = new Vector3d();
	Vector3d tempP = new Vector3d();
	Vector3d tempCalc = new Vector3d();
	Vector3d rotVelTri = new Vector3d();
	Vector3d computeVelocity = new Vector3d();
	Vector3d bestP = new Vector3d();
	RayD ray = new RayD();
	HashSet<Integer> badIdxF1 = new HashSet<>();
	HashSet<Integer> badIdxF2 = new HashSet<>();
	Forme f1;
	Forme f2;
	Forme bestPointeur = null;
	Forme bestFormeTri = null;
	double timeMin;
	int bestIdxTri;
	int bestIdxPoint;
	boolean collisionTriTri;
	CollisionPrediction pred;

	public void improvePrediction(CollisionPrediction pred, long currentTime, long dtms) {
		this.pred = pred;
		f1 = pred.forme1;
		f2 = pred.forme2;
		timeMin = 10000000;
		bestIdxTri = -1;
		bestIdxPoint = -1;
		collisionTriTri = false;

		badIdxF1.clear();
		badIdxF1.addAll(f1.joint.getIdx());
		badIdxF2.clear();
		badIdxF2.addAll(f2.joint.getIdx());
		// / END ???

		// create new dir
		Vector3d vitesseF1 = f2.vitesse.negate().addLocal(f1.vitesse);
		// ray.setDirection(vitesseF1);
		System.out.println("--------------- rayCollision ---------- " + vitesseF1);
		System.out.println("between " + f1 + " and " + f2);

		checkTriPoint(f1, f2, vitesseF1, dtms);

		System.out.println("--------------- rayCollision 2 ---------- " + vitesseF1);
		System.out.println("between " + f2 + " and " + f1);

		checkTriPoint(f2, f1, vitesseF1.negateLocal(), dtms);

		System.out.println("--------------- triBB collision ---------- ");
		checkTriTri(f2, f1, vitesseF1, dtms);

		System.out.println("--------------- END rayCollision ---------- ");

		if (bestPointeur != null) {
			// pred.init = true;
			// pred.formePoint = bestPointeur;
			// pred.formeTri = bestFormeTri;
			// pred.pointIdx = bestIdxPoint;
			// pred.triIdx = bestIdxTri;
			// pred.bestP = bestP;
			// compute time
			pred.moment = currentTime + (long)timeMin; //(long) (distMin / vitesseF1.length());

			// really, i can't be sure: maybe a tri-tri collision occur way
			// before this coll.
			// TODOAFTER use dist between OP and roundBB (max value from the 2
			// forme)
			if(collisionTriTri){
				//TODOAFTER think of a better value than that
				pred.maxMomentError = dtms/10;
				pred.maxMomentError += 
						(bestPointeur.triangles.get(bestIdxPoint).bbRound
						+ bestFormeTri.triangles.get(bestIdxTri).bbRound) / vitesseF1.length();
			}else{
				//really, it's a bit more than 0 if we are in rotation.
				//TODOAFTER think of a better value than that
				pred.maxMomentError = dtms/10;
			}

			pred.precisionMoment = Precision.REFINED;
		}
		else {
			//no collision?
			pred.precisionMoment = Precision.CANTFIND;
		}

	}

	//TODO: check que cette collision n'est aps déja été détecté par tri-point
	// ie: ne pas prendre en compte les associations tri-tri dans lequels
	//		il y a eu des interractions tri-point d'un tri de détecté.
	// En fait, il faudrait plutot merger cette boucle dans les 2 autres
	// 		a chaque triangle, on garde al liste des ids des points touchés
	//		et pour chaque triangle n'ayant pas de points qui m'on touché, on fait la détection tri/tri
	private void checkTriTri(Forme fTri1, Forme fTri2, Vector3d vitesseF1toF2, long dtms) {
		Vector3d center1 = new Vector3d();
		Vector3d center2 = new Vector3d();
		Vector3d dist1to2 = new Vector3d();
		Vector3d t1to2Speed = f2.vitesse.negate().addLocal(f1.vitesse);
		for (int i1 = 0; i1 < fTri1.triangles.size(); i1++) {
			Triangle tri1 = fTri1.triangles.get(i1);
			center1 = fTri1.transfoMatrix.mult(tri1.center);
			rotVelTri.set(fTri1.vangulaire).crossLocal(tempCalc.set(center1).subtractLocal(fTri1.position));
			for (int i2 = 0; i2 < fTri2.triangles.size(); i2++) {
				Triangle tri2 = fTri2.triangles.get(i2);
				center2 = fTri2.transfoMatrix.mult(tri2.center);
				//dist between 2 tri
//				double dist = center1.distance(center2);
				dist1to2.set(center2).subtract(center1);
				//compute speed
				computeVelocity.set(fTri2.vangulaire).crossLocal(tempCalc.set(center2).subtractLocal(fTri2.position)).negateLocal();
				computeVelocity.addLocal(t1to2Speed).addLocal(rotVelTri);
				//compute dist parcouru dans le sens qui les separe
				double distParc = computeVelocity.dot(tempCalc.set(dist1to2).normalizeLocal());
				distParc = distParc * dtms;
				
				//check if collision occur
				if(distParc > dist1to2.length()-(tri1.bbRound+tri2.bbRound)){
					double timeColl = dtms * (dist1to2.length()-(tri1.bbRound+tri2.bbRound)) / distParc;
					if (timeColl < timeMin) {
						System.out.println("1find a futur intersec  (tri-tri)");
						System.out.println("dist: "+dist1to2.length()+", time: "+timeColl);
						timeMin = timeColl;
						bestP.set(tempCalc);
						bestPointeur = fTri1;
						bestIdxPoint = i1;
						bestFormeTri = fTri2;
						bestIdxTri = i2;
						collisionTriTri = true;
					}
				}
				
			}
		}

	}

	//TODO do not use dist but time !
	private final void checkTriPoint(Forme pointeur, Forme formeTri, Vector3d p2tSpeed, long dtms) {
		// check pointe from f1
		for (int idxTri = 0; idxTri < formeTri.triangles.size(); idxTri++) {
			Triangle tri = formeTri.triangles.get(idxTri);
			// create the plane for this triangle
			triA = formeTri.points.get(tri.a);
			triB = formeTri.points.get(tri.b);
			triC = formeTri.points.get(tri.c);
			formeTri.transfoMatrix.mult(triA, tempA);
			formeTri.transfoMatrix.mult(triB, tempB);
			formeTri.transfoMatrix.mult(triC, tempC);

			// get the rotational velocity of the triangle
			tempCalc.set(tempA).addLocal(tempB).addLocal(tempC).divideLocal(3).subtractLocal(formeTri.position);
			rotVelTri.set(formeTri.vangulaire).crossLocal(tempCalc).negateLocal();

			for (int idxPoint = 0; idxPoint < pointeur.points.size(); idxPoint++) {
				if (badIdxF1.contains(idxPoint))
					continue;
				Vector3d pointLocalPos = pointeur.points.get(idxPoint);
				pointeur.transfoMatrix.mult(pointLocalPos, tempP);
				// System.out.println("Change point: " +idxPoint+
				// "@"+tempP+" ("+pointLocalPos+")");
				ray.setOrigin(tempP);
				// add rotationalForce
				computeVelocity.set(pointeur.vangulaire).crossLocal(tempP.subtract(pointeur.position));
				// System.out.println("vitesseF1 = "+vitesseF1+", rotVelo= "+computeVelocity);
				computeVelocity.addLocal(p2tSpeed).addLocal(rotVelTri);
				if (computeVelocity.lengthSquared() == 0)
					continue;
				ray.setDirection(tempCalc.set(computeVelocity).normalizeLocal());
				// System.out.println("1test on tri " +idxTri+
				// " (p "+idxPoint+") with ray "+ray+" "+ray.intersectWhere(tempA,
				// tempB, tempC, tempCalc));
				// System.out.println("tri = "+tempA+ " "+tempB+" "+ tempC);
				// System.out.println("intersection @"
				// +tempCalc+" dist="+ray.intersects(tempA, tempB, tempC));

				if (ray.intersectWhere(tempA, tempB, tempC, tempCalc)) {
					double dist = (tempCalc.distance(tempP));
					double timeToCollide = dist / computeVelocity.length();
					if (timeToCollide < timeMin) {
						System.out.println("1find a futur intersec on " + tempCalc + "@"
								+ (tempCalc.distance(tempP) + " in tri " + idxTri));
						System.out.println("dist: "+dist+", time: "+timeToCollide);
						System.out.println("from " + ray.getOrigin() + " (p " + idxPoint + ") with dir "
								+ ray.getDirection());

						System.out.println("vitesseP2T = " + p2tSpeed + ", rotVelo= " + computeVelocity);
						System.out.println("1test on tri " + idxTri + " (p " + idxPoint + ") with ray " + ray + " "
								+ ray.intersectWhere(tempA, tempB, tempC, tempCalc));
						System.out.println("tri = " + tempA + " " + tempB + " " + tempC);
						System.out
								.println("intersection @" + tempCalc + " dist=" + ray.intersects(tempA, tempB, tempC));
						// System.out.println("vlinear = " + vitesseF1);
						// System.out.println("vAngulTri = " + rotVelTri);
						// System.out.println("vAngulP = "
						// +
						// computeVelocity.set(pointeur.vangulaire).crossLocal(pointLocalPos.normalize()));
						timeMin = timeToCollide;
						bestP.set(tempCalc);
						bestPointeur = pointeur;
						bestFormeTri = formeTri;
						bestIdxTri = idxTri;
						bestIdxPoint = idxPoint;
						// pred.localTA = triA;
						// pred.localTB = triB;
						// pred.localTC = triC;
						System.out.println("set localP @ " + pointLocalPos);
						// pred.localP = pointLocalPos;
						// pred.worldTA.set(tempA);
						// pred.worldTB.set(tempB);
						// pred.worldTC.set(tempC);
						// pred.worldP.set(tempP);
						// pred.rayon.set(ray);
					}
				}
			}
		}
	}

}
