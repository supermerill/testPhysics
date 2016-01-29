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
	public Vector3f forceResultante = new Vector3f();
	public Vector3f pointPivot = new Vector3f();

	@Override
	public void updatePosition(long instant, long dt) {

		// //compute the sum of all force
		// //TODO remove mass... to create accel or add mass to gravity to have
		// a force
		// Vector3f grav = new Vector3f(0,-0.00000981f,0);
		// // Vector3f grav = new Vector3f(0,-9.81f,0);
		//
		//
		// //create the normal forces (via a dot)
		// Vector3f normal = f.position.toVec3f().subtractLocal(point);
		// System.out.println("normale : "+normal);
		// // Vector3f normal = point.subtract(f.position.toVec3f());
		// normal.normalizeLocal();
		// normal.multLocal(Math.abs(normal.dot(grav)));
		//
		// //compute the new sum (this vector is _|_ with the normal force)
		// Vector3f sumAccel = normal.add(grav);
		// System.out.println("grav : "+grav);
		// System.out.println("normale : "+normal);
		// System.out.println("sum : "+sumAccel);
		// System.out.println("dot : "+sumAccel.dot(normal));
		// System.out.println("0.0001 : "+0.0001f);
		// if(sumAccel.dot(normal) == 0)
		// {
		// //exactly at vert. Add some D
		// normal.addLocal(new Vector3f(0.000000001f,-0.000000001f,
		// 0.000000001f));
		// sumAccel = normal.add(grav);
		// }
		// assert sumAccel.dot(normal) < 0.0001 :
		// "Error: the normal vector is not _|_ with the resultante";
		//
		// //add it as rotational force
		// float rayon = f.position.toVec3f().distance(point);
		// //on a l'acceleration en m/s² en 1 point
		// // on veut obtenir l'acceleration en rad/s²
		// //or 1 rad = 180° = pi*r metres
		// //donc (rad/s²) * pi*r = m/s²
		//
		// System.out.println("f.aangulaire : "+f.aangulaire);
		//
		// //divide per rayon because of (overly simplified) moment of inertia
		// Vector3f accelAngul =
		// f.position.toVec3f().subtractLocal(point).crossLocal(sumAccel).divideLocal((float)(f.roundBBRayon*f.roundBBRayon));
		// f.aangulaire.addLocal(accelAngul);
		// Vector3f v1 = f.position.toVec3f().subtractLocal(point);
		// System.out.println("f.position.toVec3f().subtractLocal(point) : "+v1);
		// System.out.println("sum : "+sumAccel);
		// System.out.println("cross : "+sumAccel.cross(v1));
		// System.out.println("crossN : "+sumAccel.cross(normal));
		// System.out.println("f.aangulaire : "+f.aangulaire);
		// f.vangulaire.set(0,0,0);
		// f.vitesse.set(0,0,0);
		// f.acceleration.set(0,0,0);
		// //f1.physicUpdate = false;
		// f.posAxeRot.set(f.transfoMatrix.invert().mult(point));

	}

	@Override
	public void updateForce(long instant, long dt) {
		
		//pick 1 point (from first part of bad algo?)
		
		//place at left or right var if( if left of o or right
		
		//check points in dir of O
		
		//do
			//if better point at left, replace left
			//if better point at right, replace right
			//if right or left is just plain better solo, use it solo
		// while 0 is behind and you changed right or left
		
		//if O is behind => equi
		//if only 1 point => solo rot
		//else rot 2P
		
		
	}

	//maybe unrecoverable...
	public void updateForceTooBad(long instant, long dt) {
		// gather forces

		// sum them
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
		List<Vector3f> listVector = new ArrayList<>(); // TODOAFTER mettre un
														// coup de balai
		List<Vector3f> listPoint = new ArrayList<>();
		List<Vector3f> listPos2Point = new ArrayList<>();
		Vector3f fPos = f.position.toVec3f();
		for (Vector3f pointOfContact : f.jointPose.points) {
			Vector3f vect = fPos.subtract(pointOfContact);
			// System.out.println("new vect : " + vect + " from pos " +
			// pointOfContact);*
			System.out.println("f.jointPose.points.add(new Vector3f(" + pointOfContact.x + "f, 0, " + pointOfContact.z
					+ "f));");
			vect.normalize();
			// vect = Vector3f.UNIT_XYZ.divide(vect);
			if (vect.dot(sumForces) < 0) {
				listVector.add(vect);
				listPoint.add(pointOfContact);
				listPos2Point.add(pointOfContact.subtract(fPos));
			}
		}
		if (listVector.size() == 0) {
			System.err.println("TODO: gerer le décollage d'un posé");
			// ie supprimer jointposé (this) et mettre à la place un freeflight
			return;
		}

		// pick a random vect
		int idx = 0;
		Vector3f aVect = listVector.get(idx);
		// make a plane with sumF
		// Vector3f normalePlan = sumForces.cross(/*aVect*/new
		// Vector3f(1,0,0)).normalizeLocal();
		Vector3f normalePlan = sumForces.cross(aVect).normalizeLocal();
		Vector3f normalePlan2 = sumForces.cross(normalePlan).normalizeLocal();
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
			System.out.println("normaleProjPlan:" + normaleProjPlan);
			// normale2Draw.set(sumForces.normalize());
			// normale2Draw2.set(normalePlan2.normalize());
			// normale2Draw3.set(normaleProjPlan.normalize());
			System.out.println("sumForces = " + sumForces.normalize());
			System.out.println("aVect = " + aVect);
			System.out.println("aVectN = " + aVect.normalize());
			System.out.println("normalePlan = " + normalePlan.normalize());
			System.out.println("normalePlan2 = " + normalePlan2.normalize());
			System.out.println("normaleProjPlan = " + normaleProjPlan.normalize());
			// LOL NOP
			// find the point with the lower angle (best dot/length) in the
			// plane which is normalePlan but 90° rot around sumForce
			// int idxAngleLow = 0;
			// Vector3f projeteVect = listVector.get(0).subtract(
			// normaleProjPlan.mult(normaleProjPlan.dot(listVector.get(0))));
			// // TODOAFTER / normaleProjPlan.lengthsquared suffit je pense
			// (penser
			// // à update la boucle aussi)
			// float maxAngleLow = normaleProjPlan.dot(projeteVect) /
			// projeteVect.length();
			// for (idx = 1; idx < listVector.size(); idx++) {
			// projeteVect = listVector.get(idx).subtract(
			// normaleProjPlan.mult(normaleProjPlan.dot(listVector.get(idx))));
			// System.out.println("projete : "+idxAngleLow+" : "+listVector.get(idxAngleLow));
			// float angleLow = normaleProjPlan.dot(projeteVect) /
			// projeteVect.length();
			// if (maxAngleLow < angleLow) {
			// maxAngleLow = angleLow;
			// idxAngleLow = idx;
			// }
			// }

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

			// find our first point, seek a second?
			System.out.println("idxFirst : " + idxNear + " : " + listPoint.get(idxNear));
			point1 = listPoint.get(idxNear);
			Vector3f previousP1 = point1;
			int idxNear2 = -1;
			Vector3f normalePlanP1 = null;
			do {
				previousP1 = point1;
				System.out.println("---------------------------------------------");

				// create a plane with sumForces and (po X sumForces)

				// check dir
				boolean p1IsPos = sumForces.cross(normaleProjPlan).dot(point1) >= 0;
				System.out.println("check idxFirst is pos? : " + sumForces.cross(normaleProjPlan) + " .dot " + point1
						+ " = " + sumForces.cross(normaleProjPlan).dot(point1));
				System.out.println("idxFirst is pos? : " + p1IsPos);
				System.out.println(point1 + " X " + sumForces + " = " + point1.cross(sumForces));
				// System.out.println(new Vector3f(1,0,0)+" X " + new
				// Vector3f(0,1,0)+" = "+new Vector3f(1,0,0).cross(new
				// Vector3f(0,1,0)));
				// System.out.println(new Vector3f(1,0,0)+" X " + new
				// Vector3f(0,-1,0)+" = "+new Vector3f(1,0,0).cross(new
				// Vector3f(0,-1,0)));
				// System.out.println(new Vector3f(-1,0,0)+" X " + new
				// Vector3f(0,1,0)+" = "+new Vector3f(-1,0,0).cross(new
				// Vector3f(0,1,0)));
				// System.out.println(new Vector3f(-1,0,0)+" X " + new
				// Vector3f(0,-1,0)+" = "+new Vector3f(-1,0,0).cross(new
				// Vector3f(0,-1,0)));
				if (p1IsPos) {
					normalePlanP1 = point1.cross(sumForces).normalizeLocal();
				} else {
					normalePlanP1 = sumForces.cross(point1).normalizeLocal();
				}
				System.out.println("normalePlanP1 : " + normalePlanP1);

				// Vector3f normalePlanP1 =
				// sumForces.cross(listVector.get(idxNear)).normalizeLocal();
				// if (normalePlanP1.dot(listVector.get(idxNear)) < 0) {
				// System.err.println("correct normalePlanP1 dir : " +
				// normalePlanP1 + " => " + normalePlanP1.mult(-1));
				// normalePlanP1.multLocal(-1);
				// }

				// TODO check if correct direction
				// Vector3f normalePlanSearchP2 =
				// sumForces.cross(listVector.get(idxNear)).crossLocal(sumForces).normalizeLocal();
				// if(normalePlanSearchP2.dot(Vector3f.UNIT_XYZ)<0){
				// System.out.println("correct normalePlanSearchP2 dir");
				// normalePlanSearchP2.multLocal(-1);
				// }
				// normale2Draw3.set(normalePlanSearchP2.normalize());
				//
				// // find the "farest" in "front" (normalized by distance) =>
				// ie
				// big angle
				// //where is front? wtf? neg? pos?
				// int idx2 = 0;
				// float maxRatio = 0;
				// for (idx = 0; idx < listPoint.size(); idx++) {
				// if(idx == idxNear) continue;
				// float val = listPoint.get(idx).dot(normalePlanSearchP2);
				// //FIXME: distance dans le plan de sumForces?
				// System.out.println("Idx: "+idx+" "+listPoint.get(idx)+", val: "+val+", dist : "+listPoint.get(idx).distance(point1));
				// if(maxRatio < val/listPoint.get(idx).distance(point1)){
				// System.out.println("best!");
				// maxRatio = val/listPoint.get(idx).distance(point1);
				// point2 = listPoint.get(idx);
				// idx2 = idx;
				// }
				// }

				Vector3f normalePlanSearchP2 = normalePlanP1.mult(1);
				float distanceNear2 = Float.MAX_VALUE;
				for (idx = 0; idx < listPoint.size(); idx++) {
					if (idx == idxNear)
						continue;
					System.out.println("check point  = " + listPoint.get(idx) + " @ "
							+ normalePlanSearchP2.dot(listPoint.get(idx)));
					float dot = normalePlanSearchP2.dot(listPoint.get(idx));
					if (dot < distanceNear2) {
						System.out.println("great!");
						distanceNear2 = dot;
						idxNear2 = idx;

					}
				}
				if (idxNear2 >= 0) {
					System.out.println("idxSecond(temp) : " + idxNear2 + " : " + listPoint.get(idxNear2));
					point2 = listPoint.get(idxNear2);

					// try to find a more front-oriented

					boolean p1p2IsPos = sumForces.cross(normalePlanP1).dot(point2.subtract(point1)) >= 0;
					System.out.println("p1p2IsPos is pos? : " + p1p2IsPos);
					if (!p1p2IsPos) {
						normalePlanSearchP2 = point2.subtract(point1).cross(sumForces).normalizeLocal();
					} else {
						normalePlanSearchP2 = sumForces.cross(point2.subtract(point1)).normalizeLocal();
					}
					System.out.println("normalePlanSearchP2 : " + normalePlanSearchP2);
					// normalePlanSearchP2 =
					// sumForces.cross(listPoint.get(idxNear).subtract(listPoint.get(idxNear2)))
					// .normalizeLocal();
					// if (normalePlanSearchP2.dot(listVector.get(idxNear)) < 0)
					// {
					// System.out.println("correct normalePlanSearchP2 dir : " +
					// normalePlanSearchP2 + " => "
					// + normalePlanSearchP2.mult(-1));
					// normalePlanSearchP2.multLocal(-1);
					// }

					distanceNear2 = 0;
					for (idx = 0; idx < listPoint.size(); idx++) {
						if (idx == idxNear || idx == idxNear2)
							continue;
						System.out.println("check point  = " + listPoint.get(idx) + " @ "
								+ normalePlanSearchP2.dot(listPoint.get(idx)));
						float dot = normalePlanSearchP2.dot(listPoint.get(idx));
						if (dot > distanceNear2) {
							System.out.println("great!");
							distanceNear2 = dot;
							idxNear2 = idx;

						}
					}

				}

				// check if one point is not sufficient
				// FIXME
				if (idxNear2 >= 0) {
//					System.out.println("idxSecond : " + idxNear2 + " : " + listPoint.get(idxNear2));
//
//					Vector3f normaleCheckP2 = sumForces.cross(listVector.get(idxNear)).crossLocal(sumForces)
//							.normalizeLocal();
//					if (normaleCheckP2.dot(listVector.get(idxNear)) < 0) {
//						System.out.println("correct normaleCheckP2 dir");
//						normaleCheckP2.multLocal(-1);
//					}
//					System.out.println("Check P2 : " + listPoint.get(idxNear2) + " -> "
//							+ listPoint.get(idxNear2).subtract(listPoint.get(idxNear)) + " dot(" + normaleCheckP2
//							+ ") => " + normaleCheckP2.dot(listPoint.get(idxNear2).subtract(listPoint.get(idxNear))));
//					if (normaleCheckP2.dot(listPoint.get(idxNear2).subtract(listPoint.get(idxNear))) > 0) {
//						System.out.println("P2 ok");
//
//						Vector3f normaleCheckP1 = sumForces.cross(listVector.get(idxNear2)).crossLocal(sumForces)
//								.normalizeLocal();
//						if (normaleCheckP1.dot(listVector.get(idxNear2)) < 0) {
//							System.out.println("correct normaleCheckP1 dir");
//							normaleCheckP1.multLocal(-1);
//						}
//						System.out.println("Check P1 : " + listPoint.get(idxNear) + " -> "
//								+ listPoint.get(idxNear).subtract(listPoint.get(idxNear2)) + " dot(" + normaleCheckP1
//								+ ") => "
//								+ normaleCheckP2.dot(listPoint.get(idxNear).subtract(listPoint.get(idxNear2))));
//						if (normaleCheckP1.dot(listPoint.get(idxNear).subtract(listPoint.get(idxNear2))) > 0) {
//							System.out.println("P1 ok");
//						} else {
//							System.out.println("P1 not in front engough!");
//							idxNear = idxNear2;
//							idxNear2 = -1;
//							point1 = listPoint.get(idxNear);
//							normalePlanP1.set(normalePlan2);
//						}
//
//					} else {
//						System.out.println("P2 not in front engough!");
//						idxNear2 = -1;
//						point2 = point1.mult(1);
//					}
				}

				System.out.println(previousP1+" =?= "+point1);
			} while (!point1.equals(previousP1));

			// case with 2 points
			if (idxNear2 >= 0) {
				point2 = listPoint.get(idxNear2);
				System.out.println("idxSecond : " + idxNear2 + " : " + point2);

				boolean p2IsPos = sumForces.cross(normalePlanP1).dot(point2) >= 0;
				System.out.println("idx2 is pos? : " + p2IsPos);
				Vector3f normalePlanP2 = null;
				if (p2IsPos) {
					normalePlanP2 = point2.cross(sumForces).normalizeLocal();
				} else {
					normalePlanP2 = sumForces.cross(point2).normalizeLocal();
				}

				normale2Draw.set(normalePlanP1);
				System.out.println("sumForces : " + sumForces);
				System.out.println("normalePlanP1 : " + normalePlanP1);
				normale2Draw2.set(normalePlanP2);
				Vector3f visualDirectionOfRot = normalePlanP2.add(normalePlanP1).multLocal(-1).normalizeLocal();
				normale2Draw3.set(visualDirectionOfRot);
				System.out.println("normalePlanP2 : " + normalePlanP2);
				System.out.println("normalePlanP1&2 : " + visualDirectionOfRot);

				// if found, it's our second point! now we our points of
				// rotation,
				// our axe and we can set the direction & amplitude of the force

				// check if the origin is really in the front of this plane (if
				// not, it's in equilibrium state!)

				if (visualDirectionOfRot.dot(point1) > 0) {
					System.out.println("EQUILIBRIUM (CHECK 2 POS)");
				} else {
					System.out.println("ROTATION AROUND 2 P");
				}

				// Vector3f rotationXforce =
				// point1.subtract(point2).crossLocal(sumForces);
				// float signTest = rotationXforce.dot(normaleProjPlan);
				// System.out.println("rotationXforce : " + rotationXforce);
				// System.out.println("signTest : " + signTest);
				// System.out.println("point1.dot(point2) : " +
				// point1.dot(point2));
				// // if( signTest<0 && ){
				// // //equilibrium!
				// // }
				// if (signTest > 0) {
				// System.out.println("EQUILIBRIUM (CHECK 2 POS)");
				// } else {
				// System.out.println("ROTATION AROUND 2 P");
				// }

			} else {
				// check if the origin is really in the front of this plane (if
				// not, it's in equilibrium state!)
				float signTest = normalePlanP1.dot(normaleProjPlan);
				System.out.println("normalePlanP1 : " + normalePlanP1);
				System.out.println("signTest : " + signTest);
				System.out.println("point1 : " + point1);
				if (signTest > 0) {
					System.out.println("EQUILIBRIUM (CHECK 1 POS)");
				} else {
					System.out.println("ROTATION AROUND 1 P");
				}
			}
		}

		// check si la nouvelle force (sur le nouvel axe de rotation) est bloqué
		// par des points de contacts
		// comme une pierre coincé en deux autres pierre à son extrémité
		// si oui, bien transmettre la résultante à ces points-là

		// transmettre la force aux différents points de contacts (si pas déjà
		// fait)

		// faire passer cette force en accel angulaire sur l'axe donnée en
		// TODO

	}

	public Vector3f point1 = new Vector3f();
	public Vector3f point2 = new Vector3f();
	public Vector3f normale2Draw = new Vector3f();
	public Vector3f normale2Draw2 = new Vector3f();
	public Vector3f normale2Draw3 = new Vector3f();

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
		for (Vector3f pointOfContact : f.jointPose.points) {
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

		// normale - sumN = force restante (fr).
		forceResultante.set(sumForces.add(v1));

		System.out.println("resultatnte : " + forceResultante + " nullable? "
				+ (sumForces.length() / 1000 > forceResultante.length()));

		if (sumForces.length() / 1000 > forceResultante.length()) {
			forceResultante.set(0, 0, 0);
			pointPivot.set(0, 0, 0);
			return;
		}

		// trouver le point de contact le plus "eloigné" dans l'axe de fr => pe
		f.getMostFarAwayPoint(forceResultante, pointPivot);

		// faire passer cette force en accel angulaire sur l'axe donnée en
		// pe.cross(fr)
		// TODO

	}

}
