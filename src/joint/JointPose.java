package joint;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;

import old.Forme;

import com.jme3.math.Plane;
import com.jme3.math.Vector3f;

public class JointPose extends Joint {

	// HashMap<Integer, JointPose> linkToOtherForme = new HashMap<>();

	// the point-joint (s)
	public ArrayList<Vector3f> points = new ArrayList<>();
	public ArrayList<Vector3f> normales = new ArrayList<>(); // pour tester :
																// affichage
//	public Vector3f forceResultante = new Vector3f();
//	public Vector3f pointPivot = new Vector3f();
	public Vector3f rotationForceVector = new Vector3f();

	//TODOAFTER: remove debug vars
	public Vector3f point1 = new Vector3f();
	public Vector3f point2 = new Vector3f();
	public Vector3f normale2Draw = new Vector3f();
	public Vector3f normale2Draw2 = new Vector3f();
	public Vector3f normale2Draw3 = new Vector3f();

	@Override
	public void updatePosition(long instant, long dt) {
		
		 //divide per rayon because of (overly simplified) moment of inertia
		 Vector3f accelAngul = f.position.toVec3f().subtractLocal(point).crossLocal(sumAccel).divideLocal((float)(f.roundBBRayon*f.roundBBRayon));
		 f.aangulaire.addLocal(accelAngul);
		 Vector3f v1 = f.position.toVec3f().subtractLocal(point);
		 System.out.println("f.position.toVec3f().subtractLocal(point) : "+v1);
		 System.out.println("sum : "+sumAccel);
		 System.out.println("cross : "+sumAccel.cross(v1));
		 System.out.println("crossN : "+sumAccel.cross(normal));
		 System.out.println("f.aangulaire : "+f.aangulaire);
		 f.vangulaire.set(0,0,0);
		 f.vitesse.set(0,0,0);
		 f.acceleration.set(0,0,0);
		 //f1.physicUpdate = false;
		 f.posAxeRot.set(f.transfoMatrix.invert().mult(point));

	}

	@Override
	public void updateForce(long instant, long dt) {
		if(this.points.size() == 1){
			//JointPonctuel
			System.err.println("joint ponctuel instead of jointpose");
			
			//TODOAFTER more optimized? check!
//		}else if(this.points.size() == 2){
//			updateForceMax2(instant, dt);
		}else if(this.points.size() >1){
			updateForceNoMax(instant, dt);
		}else{
			System.err.println("Error, "+points.size()+" points in jointpose!");
		}
	}
	
	public void updateForceNoMax(long instant, long dt) {

		// sum forces
		Vector3f sumForces = new Vector3f(0, 0, 0);
		for (Vector3f force : f.forces) {
			sumForces.addLocal(force);
		}
		System.out.println("sumForces : " + sumForces);
		if (sumForces.lengthSquared() == 0)
			return;

		Vector3f sumForcesN = sumForces.normalize();

		// ///////create normale:

		// get all vector (-PointofContact)
		// remove all vector with dot(sumF) >0 (remove normale in the same dir
		// as the sum of forces)
		List<Vector3f> listPoint = new ArrayList<>();
		Vector3f fPos = f.position.toVec3f();
		for (Vector3f pointOfContact : points) {
			Vector3f vect = fPos.subtract(pointOfContact);
			System.out.println("f.jointPose.points.add(new Vector3f(" + pointOfContact.x + "f, 0, " + pointOfContact.z
					+ "f));");
			vect.normalize();
			// vect = Vector3f.UNIT_XYZ.divide(vect);
			if (vect.dot(sumForcesN) < 0) {
				// listVector.add(vect);
				listPoint.add(pointOfContact);
			}
		}
		if (listPoint.size() == 0) {
			System.err.println("TODO: gerer le décollage d'un posé");
			// ie supprimer jointposé (this) et mettre à la place un freeflight
			return;
		}

		// pick a random vect
		int idx = 0;
		Vector3f aVect = listPoint.get(idx);
		// make a plane with sumF
		// Vector3f normalePlan = sumForces.cross(/*aVect*/new
		// Vector3f(1,0,0)).normalizeLocal();
		Vector3f normalePlan = sumForcesN.cross(aVect).normalizeLocal();
		Vector3f normalePlan2 = sumForcesN.cross(normalePlan).normalizeLocal();
		System.out.println("normalePlan:" + normalePlan);
		System.out.println("normalePlan2:" + normalePlan2);
		// it has points in the two sides?
		boolean findPosPos = false, findPosNeg = false, findNegPos = false, findNegNeg = false;
		for (idx = 0; idx < listPoint.size() && !(findPosPos && findPosNeg && findNegPos && findNegNeg); idx++) {
			float dot = normalePlan.dot(listPoint.get(idx));
			float dot2 = normalePlan2.dot(listPoint.get(idx));
			System.out.println("Find node " + listPoint.get(idx) + " @ " + dot + " " + dot2);
			if (dot < 0) {
				if (dot2 < 0) {
					findNegNeg = true;
					System.out.println("NN");
				} else {
					findNegPos = true;
					System.out.println("NP");
				}
			} else {
				if (dot2 < 0) {
					findPosNeg = true;
					System.out.println("PN");
				} else {
					findPosPos = true;
					System.out.println("PP");
				}
			}
		}
		System.out.println("NN:" + findNegNeg + " <0< NP:" + findNegPos + " PN:" + findPosNeg + " < PP:" + findPosPos);

		if (findNegNeg && findNegPos && findPosNeg && findPosPos) {
			System.out.println("equilibrium => ok");

			// TODO: répartir les forces et les passer aux objets suivants.

		} else {
			System.out.println("equilibrium => PAS VRAI");

			Vector3f normaleProjPlan = new Vector3f();
			if (!findNegNeg) {
				normaleProjPlan.addLocal(normalePlan).addLocal(normalePlan2);
			}
			if (!findNegPos) {
				normaleProjPlan.addLocal(normalePlan).subtractLocal(normalePlan2);
			}
			if (!findPosNeg) {
				normaleProjPlan.subtractLocal(normalePlan).addLocal(normalePlan2);
			}
			if (!findPosPos) {
				normaleProjPlan.subtractLocal(normalePlan).subtractLocal(normalePlan2);
			}
			normaleProjPlan.normalizeLocal();
			normale2Draw3 = normaleProjPlan;
			System.out.println("normaleProjPlan=" + normaleProjPlan);

			// pick 1 point (from first part of bad algo?)
			// find the first point in the back of this plane
			int idxNear = 0;
			System.out.println("check point  = " + listPoint.get(0) + " @ " + normaleProjPlan.dot(listPoint.get(0)));
			float distanceNear = normaleProjPlan.dot(listPoint.get(0));
			for (idx = 1; idx < listPoint.size(); idx++) {
				System.out.println("check point  = " + listPoint.get(idx) + " @ "
						+ normaleProjPlan.dot(listPoint.get(idx)));
				float dot = normaleProjPlan.dot(listPoint.get(idx));
				if (dot < distanceNear) {
					System.out.println("great!");
					distanceNear = dot;
					idxNear = idx;
				}
			}

			// place at left or right var if( if left of o or right)
			Vector3f solo = listPoint.get(idxNear);
			Vector3f right = null;
			Vector3f left = null;
			Vector3f previousRight = null;
			Vector3f previousLeft = null;
			Vector3f previousSolo = null;
			Vector3f dirRight = new Vector3f();
			Vector3f dirCheck = new Vector3f();

			boolean pIsRight = sumForcesN.cross(normaleProjPlan).dot(solo) >= 0;
			System.out.println("Dit right: "+sumForcesN.cross(normaleProjPlan));
			System.out.println("is Right? "+pIsRight);
			if (pIsRight) {
				right = solo;
			} else {
				left = solo;
			}

			// init

			// if(right == null || left == null){
			// System.out.println("solo dir "+right != null+" from "+solo);
			// if (right != null) {
			// // dirRight = solo.cross(sumForcesN);
			// bestRight =
			// } else {
			// // dirRight = sumForcesN.cross(solo).mult(-1);
			// }
			dirRight = sumForcesN.cross(solo);
			System.out.println("solo: " + solo);
			System.out.println("sumForcesN: " + sumForcesN);
			System.out.println("dirRight: " + dirRight);
			// }
			// else{
			// dirRight = right.subtract(left);
			// System.out.println("duo dir right : "+dirRight);
			// }
			dirCheck = sumForcesN.cross(dirRight).normalizeLocal();

			// do
			boolean equilibre = false;
			do {
				previousRight = right;
				previousLeft = left;
				previousSolo = solo;
				System.out.println("begin boucl!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ");
				System.out.println("solo " + solo);
				System.out.println("right " + right);
				System.out.println("left " + left);
				float bestRight = -Float.MAX_VALUE;
				float bestLeft = -Float.MAX_VALUE;
				// check points in dir of O
				if (right != null) {
					bestRight = dirCheck.dot(right);
				}
				if (left != null) {
					bestLeft = dirCheck.dot(left);
				}
				System.out.println("bestRight : " + bestRight);
				System.out.println("bestLeft : " + bestLeft);
				System.out.println("dir to right : " + dirRight);
				System.out.println("dir to check : " + dirCheck);
				for (idx = 0; idx < listPoint.size(); idx++) {
					Vector3f aPoint = listPoint.get(idx);
					if (aPoint == right || aPoint == left)
						continue;
					// check what side
					float side = dirRight.dot(aPoint);
					float dist = dirCheck.dot(aPoint);
					System.out
							.println("check point  = " + listPoint.get(idx) + ", side = " + side + ", dist = " + dist);
					if (side >= 0) {
						// if better point at right, replace right
						if (dist > bestRight) {
							System.out.println("new right!");
							bestRight = dist;
							right = aPoint;
							solo = aPoint;
						}
					} else {
						// if better point at left, replace left
						if (dist > bestLeft) {
							System.out.println("new left!");
							bestLeft = dist;
							left = aPoint;
							solo = aPoint;
						}
					}
				}
				System.out.println("new right: " + right);
				System.out.println("new left: " + left);
				// if right or left is just plain better solo, use it solo
				if (right != null && left != null) {
					System.out.println("right dir: " + sumForcesN.cross(sumForcesN.cross(right)));
					System.out.println("right dir dot left: " + sumForcesN.cross(sumForcesN.cross(right)).dot(left.subtract(right)));
					if (sumForcesN.cross(sumForcesN.cross(right)).dot(left.subtract(right)) < 0) {
						System.out.println("right better than left");
						left = null;
						solo = right;
					} else {
						System.out.println("left dir: " + sumForcesN.cross(sumForcesN.cross(left)));
						System.out.println("left dir dot left: " + sumForcesN.cross(sumForcesN.cross(left)).dot(right.subtract(left)));
						if (sumForcesN.cross(sumForcesN.cross(left)).dot(right.subtract(left)) < 0) {
							System.out.println("left better than right");
							right = null;
							solo = left;
						}
					}
				}

				// while 0 is behind and you changed right or left
				System.out.println("verify pos of O (via this dir: " + normaleProjPlan + ")");
				if (right != null && left != null) {
					dirRight = right.subtract(left);
					System.out.println("new dirRight : "+dirRight);
					dirCheck = sumForcesN.cross(dirRight).normalizeLocal();
					System.out.println("new dircheck : "+dirCheck);
//					solo = right.add(left).multLocal(0.5f);
					System.out.println("verify if vector is not in the right side of O : " + 
							normaleProjPlan.dot(dirCheck));
					System.out.println("verify if we have not dépassé O : " + 
							dirCheck.dot(right.add(left).multLocal(0.5f)));
					equilibre = normaleProjPlan.dot(dirCheck) > 0 || dirCheck.dot(right.add(left).multLocal(0.5f)) > 0;
					if (!equilibre) {
						// check no points are in front of the 'line'
						for (idx = 0; idx < listPoint.size(); idx++) {
							Vector3f aPoint = listPoint.get(idx);
							if (aPoint == right || aPoint == left)
								continue;
							System.out.println("check point if in front of " + dirCheck + "  : " + aPoint);
							if (aPoint.dot(dirCheck) > 0) {
								System.out.println("find!");
								equilibre = true;
							}
						}
					}
				} else {
					System.out.println("verify if solo is behind O : " + normaleProjPlan.dot(solo));
					equilibre = normaleProjPlan.dot(solo) < 0;
					if (!equilibre) {
						dirRight = sumForcesN.cross(solo);
						dirCheck = sumForcesN.cross(dirRight).normalizeLocal();
						for (idx = 0; idx < listPoint.size(); idx++) {
							Vector3f aPoint = listPoint.get(idx);
							if (aPoint == right || aPoint == left)
								continue;
							System.out.println("check point if in front of " + dirCheck + "  : " + aPoint);
							if (aPoint.dot(dirCheck) > 0) {
								System.out.println("find!");
								equilibre = true;
							}
						}
					}
				}

				System.out.println("boucle test " + equilibre + " && " + previousRight +"==" +right
						+ " && " + previousLeft +"=="+ left + " && " + previousSolo +"==" +solo);
			} while (!equilibre && !(previousRight == right && previousLeft == left && previousSolo == solo));
			System.out.println("boucle finie -------------------- " + equilibre + " && " + (previousRight == right)
					+ " && " + (previousLeft == left) + " && " + (previousSolo == solo));

			if (equilibre) {
				// if O is behind => equi
				System.out.println("EQUILIBRE");
				point1 = point2 = Vector3f.ZERO;
				rotationForceVector.set(0,0,0);
			} else if (right == null || left == null) {
				// if only 1 point => solo rot
				System.out.println("ROT 1P " + solo);
				point1 = point2 = solo;
				rotationForceVector.set(solo).normalizeLocal().crossLocal(sumForces);
			} else {
				// else rot 2P
				System.out.println("ROT 2P " + right + " , " + left);
				point1 = right;
				point2 = left;
				//projection of O on plane (p1;p2) X SUM
//				Plane plane = new Plane();
//				plane.setPlanePoints(point1, point2, point2.add(sumForcesN));
//				plane.setOriginNormal(point1, normal);

				rotationForceVector.set(left).subtractLocal(right).normalizeLocal()
					.mult(sumForces.length());
			}

		}

	}

	// PASBON sauf nbPoints == 2 (et peut-etre 1, pas testé
	public void updateForceMax2(long instant, long dt) {
		// gather forces

		// sum them
		Vector3f sumForces = new Vector3f(0, 0, 0);
		for (Vector3f force : f.forces) {
			sumForces.addLocal(force);
		}
		System.out.println("sumForces : " + sumForces);
		if (sumForces.lengthSquared() == 0)
			return;

		// ///////create normale:

		// get all vector (-PointofContact)
		// remove all vector with dot(sumF) >0 (remove normale in the same dir
		// as the sum of forces)
		List<Vector3f> listVector = new ArrayList<>();
		Vector3f fPos = f.position.toVec3f();
		for (Vector3f pointOfContact : points) {
			Vector3f vect = fPos.subtract(pointOfContact);
			System.out.println("new vect : " + vect + " from pos " + pointOfContact);
			if (vect.dot(sumForces) < 0) {
				listVector.add(vect);
			}
		}
		if (listVector.size() == 0) {
			System.err.println("TODO: gerer le décollage d'un posé");
			return;
		}

		sumForces.multLocal(-1);
		ArrayList<Vector3f> pasVu = new ArrayList<>(listVector);
		List<Vector3f> dejaVu = new ArrayList<>();
		// mettre un vecteur dans deja vu avec la norme de sum
		Vector3f /* sumDejaVu */v1 = new Vector3f(pasVu.remove(pasVu.size() - 1));
		System.out.println("v1 = " + v1);
		// set norm to H
		// System.out.println("sumForces.length() = "+sumForces.length());
		// System.out.println("sumForces.normalize().dot(v1) = "+sumForces.normalize().dot(v1));
		v1.multLocal(sumForces.length() / ((float) (sumForces.normalize().dot(v1))));
		System.out.println("v1 init = " + v1);
		dejaVu.add(new Vector3f(v1));
		// Boucle
		while (pasVu.size() > 0) {
			System.out.println("--------------- iteration resultante --------------" + v1);
			// v1 = sumDejaVu
			// prendre 1 nouveau vector v2 et creer le plan v1, v2
			Vector3f v2 = pasVu.remove(pasVu.size() - 1);
			// Plane plan = new Plane(sumDejaVu.cross(v2), 0);
			Vector3f normalePlane = v1.cross(v2).normalizeLocal();
			System.out.println("v1 debut = " + v1);
			System.out.println("v2 debut = " + v2);
			System.out.println("normale = " + normalePlane);

			// projecter sum sur ce plan => vh
			Vector3f vh = sumForces.subtract(normalePlane.mult(normalePlane.dot(sumForces)));
			Vector3f vhNormalized = vh.normalize();

			// calculer l'angle de v1 par rapport à vh dans le plan => a1
			double a1 = Math.acos(v1.dot(vhNormalized) / v1.length());

			// calculer l'angle de v2 par rapport à vh dans le plan => a2
			double a2 = Math.acos(v2.dot(vhNormalized) / v2.length());
			System.out.println("a1 = " + a1 * 180 / Math.PI);
			System.out.println("a2 = " + a2 * 180 / Math.PI);

			System.out.println("vh.length( = " + vh.length());
			System.out.println("Math.sin(a1) = " + Math.sin(a1));
			System.out.println("Math.tan(a1) = " + Math.tan(a1));
			System.out.println("(1/Math.tan(a1)+1/Math.tan(a2)) = " + (1 / Math.tan(a1) + 1 / Math.tan(a2)));
			System.out.println("(1/Math.tan(a1)+1/Math.tan(a2)) = " + (1 / Math.tan(a1) + 1 / Math.tan(a2)));
			// calculer la norme de v1 tel que n(v1) = n(vh) *
			// (tan(a1)+tan(a2))/sin(a1)
			double lengthv1 = (vh.length() / (1 / Math.tan(a1) + 1 / Math.tan(a2)) / Math.sin(a1));
			System.out.println("lengthv1 = " + lengthv1);
			// calculer la norme de v2 tel que n(v2) = n(vh) *
			// (tan(a1)+tan(a2))/sin(a2)
			double lengthv2 = (vh.length() / (1 / Math.tan(a1) + 1 / Math.tan(a2)) / Math.sin(a2));
			System.out.println("lengthv2 = " + lengthv2);
			// mult tous les vecteurs de dejavu par n(v1) / n(v1OLD)
			for (Vector3f ancienVect : dejaVu) {
				System.out.println("dejavu = " + ancienVect + " * "
						+ ancienVect.mult((float) (lengthv1 / (ancienVect.length()))));
				ancienVect.multLocal((float) (lengthv1 / (ancienVect.length())));
				System.out.println("dejavu => " + ancienVect);

			}
			// ajouter v2 a la liste des dejavu (et v1 si première iteration)
			v2.multLocal((float) (lengthv2 / (v2.length())));
			dejaVu.add(v2);
			System.out.println("v2 => " + v2);
			// utiliser v1+v2 dans la prochaine boucle en tant que v1
			v1.multLocal((float) (lengthv1 / (v1.length())));
			v1.addLocal(v2);
		}
		// endboucle
		sumForces.multLocal(-1);

		// // end create normale (ou sumN)
		normales.clear();
		for (int i = dejaVu.size() - 1; i >= 0; i--) {
			normales.add(dejaVu.get(i));
		}

//		// normale - sumN = force restante (fr).
//		forceResultante.set(sumForces.add(v1));
//
//		System.out.println("resultatnte : " + forceResultante + " nullable? "
//				+ (sumForces.length() / 1000 > forceResultante.length()));
//
//		if (sumForces.length() / 1000 > forceResultante.length()) {
//			forceResultante.set(0, 0, 0);
//			pointPivot.set(0, 0, 0);
//			return;
//		}
//
//		// trouver le point de contact le plus "eloigné" dans l'axe de fr => pe
//		f.getMostFarAwayPoint(forceResultante, pointPivot);

		// faire passer cette force en accel angulaire sur l'axe donnée en
		// pe.cross(fr)
		// TODO

	}

}
