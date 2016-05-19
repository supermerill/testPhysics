package collision;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;

import jme3Double.Matrix4d;
import jme3Double.Quaterniond;
import jme3Double.Vector3d;
import joint.Joint;
import joint.JointPonctuel;
import joint.JointPose;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Matrix4f;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;
import com.jme3.math.Plane.Side;

import old.Forme;
import old.Triangle;
import sun.security.util.Length;

public class CollisionUpdater {

	// prediction of collision, list to re-check (because of rotation, for
	// incoming other objects, near the predicted object)
	public Map<Long, CollisionPrediction> toCheck = new HashMap<>();

	// this forme has suffer external force (en plus de la gravite) => recheck
	// prediction
	public Collection<Forme> allFormes = new ArrayList<>();
	public Collection<CollisionPrediction> predictions = new ArrayList<>();

	// public Collection<Forme> toUpdate = new ArrayList<>();

	// TODO: colision avec forme jointés
	public void updateCollision(long currentTime, long dt) {
		float dts = dt * 0.001f;
		// F: update list of future collision
		// ie check bb for mouving elems &elems with new forces
		for (Forme f1 : allFormes) {
			System.out.println("check forme "+f1+" : "+f1.physicUpdate+f1.position);
			if (f1.physicUpdate) {
				for (Forme f2 : allFormes) {
					if (f1 == f2)
						continue;
					System.out.println("with forme "+f2+" : "+f2.physicUpdate+f2.position);
					System.out.println("dist: " + f1.position.distance(f2.position) + " ? "
							+ (f1.roundBBRayon + f2.roundBBRayon) + "+" + (f1.vitesse.length() + f2.vitesse.length())
							* dts);
					//TODOAFTER: instead of check if near, check if it can collide with speed?
					if (f1.position.distance(f2.position) < f1.roundBBRayon*2 + f2.roundBBRayon*2
							+ (f1.vitesse.length() + f2.vitesse.length()) * dts) {
						System.out.println("possible collision " + f1.predictions.get(f2));
						// check if collision exist (and it's not a joint)
						CollisionPrediction collider = f1.predictions.get(f2);
						if (collider == null /* && !f1.joint.containsKey(f2) */) {
							System.out.println("poCreate ssible collision ");
							// create a collisionPrediction
							collider = new CollisionPrediction();
							collider.init = false;
							collider.formePoint = f1;
							collider.formeTri = f2;
							f1.predictions.put(f2, collider);
							f2.predictions.put(f1, collider);
							predictions.add(collider);
						}
						collider.finded = true;

					}
				}
			}
		}
		// TODO: check aussi pour les objets rapide, quand la bb tapera une
		// autre bb

		// N : check list of nearly instant collision
		// ie elem with bb collided
		Iterator<CollisionPrediction> itPred = predictions.iterator();
		while (itPred.hasNext()) {
			CollisionPrediction pred = itPred.next();
			if (!pred.finded) {
				// remove old ones
				pred.formePoint.predictions.remove(pred.formeTri);
				pred.formeTri.predictions.remove(pred.formePoint);
				itPred.remove();
			} else {
				// check next level
				if (!pred.init) {
					System.out.println("INIT collision ");
					initPrediction(pred, currentTime);
				} else {
					System.out.println("UPDATE collision ");
					updatePrediction(pred, currentTime);
				}
				// check if prediction will be realized here
				if (pred.init) {
					checkPredictionRealization(pred, currentTime, dt);
				}
			}
		}

		// U: now update position of all elems (special update for collided
		// objects)
		for (Forme f : allFormes) {
			updatePos(f, currentTime, dt);
		}

		// G: upadte forces ?

		// /\ urgent stuff | loose stuff \/ /////////////////////////////

		// M: update collision predictions => check if, with rotations and other
		// things, p&t has changed

		// E: check if via external forces, some obects moves, deform and other
		// things.

		// S: Integrate fast moving object with rotation more than other
		// objects!

		// D: delete geometry in the same plane, or almost and too close
		// together.
		// C: create geometry when a triangle is too large

	}

	private void updatePos(Forme f, long currentTime, long dtms) {
		float dts = dtms * 0.001f;
		// System.out.println("updatePos"+f.name);
		// System.out.println(f+" update pos with v="+f.vitesse+", av="+f.vangulaire);
		if (f.vitesse.lengthSquared() + f.vangulaire.lengthSquared() == 0) {
			// don't move => don't collide
			f.joint.updatePosition(currentTime, dtms);
			

			// TODO: check if this should done here or at other points
//
//			// update linear speed
//			f.calculF.set(f.lastAccel).multLocal(dts * 0.5f);
//			f.vitesse.addLocal(f.calculF);
//			f.calculF.set(f.acceleration).multLocal(dts * 0.5f);
//			f.vitesse.addLocal(f.calculF);
//			System.out.println(f.name+"now speed = " + f.vitesse);
//
//			// update angular speed
//			f.calculF.set(f.lastAangulaire).multLocal(dts * 0.5f);
//			f.vangulaire.addLocal(f.calculF);
//			f.calculF.set(f.aangulaire).multLocal(dts * 0.5f);
//			f.vangulaire.addLocal(f.calculF);
//			System.out.println(f.name+"now aspeed = " + f.vangulaire);
		} else {

			// check number of collision
			if (f.collideAt.size() == 0) {
				// free flight
				f.joint.updatePosition(currentTime, dtms);

				// move "normally" (integrate by half)
				System.out.println("vitesse " + f.vitesse);
				System.out.println("dt " + dtms);
				System.out.println("position " + f.position);
				//
				// System.out.println("dt²"+dt * dt);
				// System.out.println("dt/1000"+dt * 0.001);
				// System.out.println("dt*dt/1000"+dt *dt * 0.001);
				// System.out.println("(dt/1000)²"+dt *dt * 0.000001);
				// System.out.println("grav(dt/1000)²"+9.81*dt *dt * 0.000001);
				// System.out.println("grav(dt²/1000)"+0.00981*dt *dt * 0.001);
				// move linear

				// if (f.posAxeRot.lengthSquared() == 0) {
//	 //used			f.calculF.set(f.vitesse).multLocal(dts); //use
//	 //used			f.position.addLocal(f.calculF);
				//it's better do to this but it render the colision prediction unreliable
				//and it's very small anyway
//				f.calculF.set(f.lastAccel).multLocal(dts * dts);
//				f.position.addLocal(f.calculF);
				// }
				System.out.println("new position " + f.position);

				// move angular
				// Matrix3f matriceRot = new Matrix3f(Matrix3f.IDENTITY);
				// matriceRot.multCACACPABONLocal(f.vangulaire);
				// Matrix4f matriceAdd = new Matrix4f();
				// matriceAdd.setTransform(f.calculF, Vector3f.UNIT_XYZ,
				// matriceRot);
				// f.transfoMatrix.addLocal(matriceAdd);
				// keep quaternion? => it reduce rounding error
	// //used			f.pangulaire.fromRotationMatrix(f.transfoMatrix.toRotationMatrix());
				// float angle = f.vangulaire.length();
				// if (angle > 0.0) // the formulas from the link
				// {
				// f.pangulaire.addLocal(new Quaternion(
				// f.vangulaire.x * FastMath.sin(angle / 2.0f) / angle,
				// f.vangulaire.y * FastMath.sin(angle / 2.0f) / angle,
				// f.vangulaire.z * FastMath.sin(angle / 2.0f) / angle,
				// FastMath.cos(angle / 2.0f)));
				// }
//				Quaternion previousAngle = new Quaternion(f.pangulaire);
//		 //used		Quaternion quaterAdd = new Quaternion().fromAngleAxis(f.vangulaire.length() * dts, f.vangulaire);
//	 //used			f.pangulaire.multLocal(quaterAdd);
//	 //used			f.pangulaire.normalizeLocal();

				// update linear speed
//	//used				f.calculF.set(f.lastAccel).multLocal(dts * 0.5f);
//		//used			f.vitesse.addLocal(f.calculF);
//	//used				f.calculF.set(f.acceleration).multLocal(dts * 0.5f);
//	//used				f.vitesse.addLocal(f.calculF);
				// System.out.println("last accel = "+f.lastAccel);
				// System.out.println("current accel = "+f.acceleration);
//	//used				System.out.println("now new speed = " + f.vitesse);

				// update angular speed
//	//used				f.calculF.set(f.lastAangulaire).multLocal(dts * 0.5f);
//	//used				f.vangulaire.addLocal(f.calculF);
//	//used				f.calculF.set(f.aangulaire).multLocal(dts * 0.5f);
//	//used				f.vangulaire.addLocal(f.calculF);
//	//used				System.out.println("now new aspeed = " + f.vangulaire);

				// more precise!

				// f.pangulaireP.multLocal(new
				// Quaterniond().fromAngleAxis(f.vangulaire.length() * dt, new
				// Vector3d(f.vangulaire)));
				// f.pangulaireP.normalizeLocal();

				// if (f.posAxeRot.lengthSquared() != 0) {
				// // move forme to rot pos
				// //MAYTODO: find a better way with rot mat
				// //http://www.euclideanspace.com/maths/geometry/affine/aroundPoint/index.htm
				//
				// Matrix4d newRot = new Matrix4d();
				// newRot.setTransform(Vector3d.ZERO, Vector3d.UNIT_XYZ,
				// new Quaterniond(quaterAdd).toRotationMatrix());
				//
				// Matrix4d preciseTrsf = new Matrix4d();
				// preciseTrsf.setTransform(f.position, Vector3d.UNIT_XYZ,
				// previousAngle.toRotationMatrix());
				//
				// Vector3d transl = new Vector3d(f.posAxeRot).mult(1);
				// // new Matrix4d(f.transfoMatrix).multNormal(transl, transl);
				// preciseTrsf.multNormal(transl, transl);
				// newRot.m03 = transl.x - newRot.m00 * transl.x - newRot.m01 *
				// transl.y - newRot.m02
				// * transl.z;
				// newRot.m13 = transl.y - newRot.m10 * transl.x - newRot.m11 *
				// transl.y - newRot.m12
				// * transl.z;
				// newRot.m23 = transl.z - newRot.m20 * transl.x - newRot.m21 *
				// transl.y - newRot.m22
				// * transl.z;
				// System.out.println("Translation : "+newRot.toTranslationVector());
				//
				// //f.transfoMatrix.translateVect(newRot.toTranslationVector());
				// // System.out.println("previousPs: "+f.position);
				// f.position.addLocal(newRot.toTranslationVector());
				// // System.out.println("correctedPs: "+f.position);
				// // f.transfoMatrix.set(newRot.multLocal(f.transfoMatrix));
				// Vector3d lastPosPoint = preciseTrsf.mult(new
				// Vector3d(f.points.get(f.points.size()-1)));
				// System.out.println("Point pos: "+preciseTrsf.mult(new
				// Vector3d(f.points.get(f.points.size()-1))));
				// //as it's an integration =>notlinear, discrete (via dt),
				// replace the point at the good place
				//
				// preciseTrsf.setTransform(f.position, Vector3d.UNIT_XYZ,
				// new Quaterniond(f.pangulaire).toRotationMatrix());
				//
				// Vector3d newPosPoint = preciseTrsf.mult(new
				// Vector3d(f.points.get(f.points.size()-1)));
				// f.position.addLocal(lastPosPoint.subtractLocal(newPosPoint));
				//
				// }

				// this was moved to joint
				// if (f.posAxeRot.lengthSquared() != 0) {
				// // move forme to rot pos
				// //MAYTODO: find a better way with rot mat
				// //http://www.euclideanspace.com/maths/geometry/affine/aroundPoint/index.htm
				//
				// Matrix4f newRot = new Matrix4f();
				// newRot.setTransform(Vector3f.ZERO, Vector3f.UNIT_XYZ,
				// quaterAdd.toRotationMatrix());
				//
				// Matrix4f preciseTrsf = new Matrix4f();
				// preciseTrsf.setTransform(f.position.toVec3f(),
				// Vector3f.UNIT_XYZ,
				// previousAngle.toRotationMatrix());
				//
				// Vector3f transl = new Vector3f(f.posAxeRot).mult(1);
				// // new Matrix4d(f.transfoMatrix).multNormal(transl, transl);
				// preciseTrsf.multNormal(transl, transl);
				// newRot.m03 = transl.x - newRot.m00 * transl.x - newRot.m01 *
				// transl.y - newRot.m02
				// * transl.z;
				// newRot.m13 = transl.y - newRot.m10 * transl.x - newRot.m11 *
				// transl.y - newRot.m12
				// * transl.z;
				// newRot.m23 = transl.z - newRot.m20 * transl.x - newRot.m21 *
				// transl.y - newRot.m22
				// * transl.z;
				// System.out.println("Translation : "+newRot.toTranslationVector());
				//
				// //f.transfoMatrix.translateVect(newRot.toTranslationVector());
				// // System.out.println("previousPs: "+f.position);
				// Vector3f newSpeed = newRot.toTranslationVector();
				// f.position.addLocal(newSpeed);
				// // System.out.println("correctedPs: "+f.position);
				// // f.transfoMatrix.set(newRot.multLocal(f.transfoMatrix));
				// Vector3f lastPosPoint = preciseTrsf.mult(new
				// Vector3f(f.points.get(f.points.size()-1)));
				// System.out.println("Point pos: "+preciseTrsf.mult(new
				// Vector3f(f.points.get(f.points.size()-1))));
				// //as it's an integration =>notlinear, discrete (via dt),
				// replace the point at the good place
				//
				// preciseTrsf.setTransform(f.position.toVec3f(),
				// Vector3f.UNIT_XYZ,
				// f.pangulaire.toRotationMatrix());
				//
				// Vector3f newPosPoint = preciseTrsf.mult(new
				// Vector3f(f.points.get(f.points.size()-1)));
				// newSpeed.addLocal(lastPosPoint.subtractLocal(newPosPoint));
				// f.position.addLocal(lastPosPoint);
				//
				// // DANGER! implique que il n'y a pas de translation si
				// rottion autour d'un axe!
				// //utile pour le calcul de collision
				// f.vitesse.set(newSpeed);
				// }
//	//used				f.transfoMatrix.setTransform(f.position.toVec3fLocal(f.calculF), Vector3f.UNIT_XYZ,
//	//used						f.pangulaire.toRotationMatrix());

			} else {
				
				resolveCollision(f, currentTime, dtms);
			}

		}

		// switch accel
		// Vector3f temp = f.lastAccel;
		f.lastAccel.set(f.acceleration);
		// f.acceleration = temp;
		// temp = f.lastAangulaire;
		f.lastAangulaire.set(f.aangulaire);
		// f.aangulaire = temp;

	}
	
	private void resolveCollisionIntegration(Forme f, long currentTime, long dtms) {
		float dts = dtms*0.001f;

		// collision(s), find the best!
		// ie find the first collision
		// ie min(dt) with distance - vitesse*dt = 0
		// => min(distance/vitesse)
		// mais cela peut ne pas fonctionner: objets jointés, mouvements bizarre...
		// => il faut tous les faire! et les ordonner par temps
		// => ensuite, on applique le premier
		// => et on recommence avec une liste de colision possible moins un element
		//PASSER AL LA V2 => spatio-temporel
		
		int idxCollision = 0;
		for (int i = 1; i < f.collideAt.size(); i++) {
			// TODO
		}

		CollisionPrediction pred = f.collideAt.get(idxCollision);

		Forme f1 = pred.formePoint;
		Forme f2 = pred.formeTri;
		
		Vector3f f1prevPos = f1.position.toVec3f();
		Vector3f f2prevPos = f2.position.toVec3f();
		Matrix4f f1prevMat = f1.transfoMatrix.mult(1);
		Matrix4f f2prevMat = f2.transfoMatrix.mult(1);

		Vector3f f1newPos = f1.position.toVec3f();
		Vector3f f2newPos = f2.position.toVec3f();
		Matrix4f f1newMat = f1.transfoMatrix.mult(1);
		Matrix4f f2newMat = f2.transfoMatrix.mult(1);

		long currentMs = currentTime;
		long maxMs = currentTime+dtms;
		
		
		//idea: move2predictedTime/2, check no collision, redo initCollision
		// iterate with this. When arrive at only dtms=0, 1 ou 2 alors stop
		// move "brutally" the least attach of the two to the best position (via joint object)
		


		Vector3f tempVect = new Vector3f();
		Vector3f tempA = new Vector3f();
		Vector3f tempB = new Vector3f();
		Vector3f tempC = new Vector3f();
		Vector3f tempP = new Vector3f();
		tempA = pred.formeTri.transfoMatrix.mult(pred.localTA, new Vector3f());
		tempB = pred.formeTri.transfoMatrix.mult(pred.localTB, new Vector3f());
		tempC = pred.formeTri.transfoMatrix.mult(pred.localTC, new Vector3f());
		tempP = pred.formePoint.transfoMatrix.mult(pred.localP, new Vector3f());

		Vector3f vitesseAPC = pred.formePoint.vangulaire.cross(pred.localP);
		Vector3f vitesseATC = pred.formeTri.vangulaire.cross(tempVect.set(pred.localTA).addLocal(pred.localTB)
				.addLocal(pred.localTC).divideLocal(3));
		Vector3f vitesseAP = new Vector3f(vitesseAPC);
		Vector3f vitesseAT = new Vector3f(vitesseATC);
		
		Plane planTri = new Plane();
		planTri.setPlanePoints(tempA, tempB, tempC);
		createNewPoint(pred.formeTri, pred.triIdx, pred.bestP);
		int idxNewPointInTri = pred.formeTri.points.size() -1;
		Vector3f previousBestP = new Vector3f();
		
		while(tempP.distance(pred.bestP)>0.00005f && !previousBestP.equals(pred.bestP)){
			previousBestP.set(pred.bestP);
			System.out.println("CU pred.formePoint.position : " + pred.formePoint.position);
			int possibilityJointP = pred.formePoint.joint.degreeOfLiberty();
			int possibilityJointT = pred.formeTri.joint.degreeOfLiberty();
			if (possibilityJointP > possibilityJointT) {
				pred.formePoint.joint.gotoCollision(pred.pointIdx, planTri);
			} else if (possibilityJointP > possibilityJointT) {
				pred.formeTri.joint.gotoCollision(pred.localTA, pred.localTB, pred.localTC, tempP);
			} else {
				// use the one who move faster
				if (vitesseAPC.addLocal(pred.formePoint.vitesse).length() >= vitesseATC.addLocal(
						pred.formeTri.vitesse).length()) {
					pred.formePoint.joint.gotoCollision(pred.pointIdx, planTri);
				} else {
					pred.formeTri.joint.gotoCollision(pred.localTA, pred.localTB, pred.localTC,  tempP);
				}
			}
			System.out.println("CU pred.formePoint.position AFTER : " + pred.formePoint.position);
			System.out.println("CU Point newPos5 : " + tempP);
			// now place tri at the right place (with thing as long as 200m
			// in rotation, it can create error like 0.8m)
			//now, with gotoCollision done, the error is below 0.1 mm
			Vector3f localTriP = pred.formeTri.points.get(idxNewPointInTri);
			tempA = pred.formeTri.transfoMatrix.mult(pred.localTA, new Vector3f());
			tempB = pred.formeTri.transfoMatrix.mult(pred.localTB, new Vector3f());
			tempC = pred.formeTri.transfoMatrix.mult(pred.localTC, new Vector3f());
			tempP = pred.formePoint.transfoMatrix.mult(pred.localP, new Vector3f());
			System.out.println("Point newPos6 : " + tempP);
			planTri.setPlanePoints(tempA, tempB, tempC);
			pred.bestP = planTri.getClosestPoint(tempP);
			System.out.println("set tri point from "+pred.formeTri.transfoMatrix.mult(localTriP)
					+" to "+pred.bestP+" near "+tempP+" => dist="+tempP.distance(pred.bestP)+" =?= "
					+ planTri.pseudoDistance(tempP));
			localTriP.set(pred.formeTri.transfoMatrix.invert().mult(pred.bestP));
		
		}

		createNewPoint(pred.formeTri, pred.triIdx, pred.bestP);

		System.out.println("CU add a new colision point (P): "+pred.formePoint+" : "+tempP+" == "
		+pred.formePoint.transfoMatrix.mult(pred.formePoint.points.get(pred.pointIdx)));
		pred.formePoint.joint.addCollisionPoint(tempP, pred.pointIdx, 
				pred.formeTri,  idxNewPointInTri);
		System.out.println("CU add a new colision point (T): "+pred.formeTri+" : "+pred.bestP+" == "
		+pred.formeTri.transfoMatrix.mult(pred.formeTri.points.get(idxNewPointInTri)));
		pred.formeTri.joint.addCollisionPoint(pred.bestP, idxNewPointInTri,
				pred.formePoint, pred.pointIdx);

		// TODO add energy->spedd from the current velocity ?
		System.out.println("COLLISON => REMOVE ALL SPEED");
		pred.formePoint.vangulaire.set(0, 0, 0);
		pred.formePoint.vitesse.set(0, 0, 0);
		pred.formePoint.acceleration.set(0, 0, 0);
		pred.formePoint.aangulaire.set(0, 0, 0);
		pred.formeTri.vangulaire.set(0, 0, 0);
		pred.formeTri.vitesse.set(0, 0, 0);
		pred.formeTri.acceleration.set(0, 0, 0);
		pred.formeTri.aangulaire.set(0, 0, 0);

		// clear collision, need to re-init all of them now.
		pred.formePoint.collideAt.clear();
		pred.formeTri.collideAt.clear();
		predictions.remove(pred);
		
	}
	
	
	
	
	private void resolveCollision(Forme f, long currentTime, long dtms) {
		float dts = dtms*0.001f;

		// collision(s), find the best!
		// ie find the first collision
		// ie min(dt) with distance - vitesse*dt = 0
		// => min(distance/vitesse)
		// mais cela peut ne pas fonctionner: objets jointés, mouvements bizarre...
		// => il faut tous les faire! et les ordonner par temps
		// => ensuite, on applique le premier
		// => et on recommence avec une liste de colision possible moins un element
		//PASSER AL LA V2 => spatio-temporel
		
		int idxCollision = 0;
		for (int i = 1; i < f.collideAt.size(); i++) {
			// TODO
		}

		CollisionPrediction pred = f.collideAt.get(idxCollision);
		System.out.println("Triangle forme "+pred.formeTri+" pos @: " + pred.formeTri.position);
		System.out.println("Triangle localPos: " + pred.localTA + ", " + pred.localTB + ", " + pred.localTC);
		System.out.println("Triangle previousPos: " + pred.worldTA + ", " + pred.worldTB + ", " + pred.worldTC);
		Vector3f tempVect = new Vector3f();
		

		if(pred.formeTri.joint instanceof JointPose){
			JointPose joint = (JointPose) pred.formeTri.joint;
			System.out.println("Check Coll forme tri "+pred.formeTri+" : ");
			int numPointToCheckAeff = 0;
			while (numPointToCheckAeff < joint.points.size()) {
					Vector3f point = joint.points.get(numPointToCheckAeff);
					System.out.println("Joint pose already has : " + point + ".distance("
							+ pred.formeTri.transfoMatrix.mult(pred.formeTri.points.get(joint.pointsIdx.get(numPointToCheckAeff))) 
							+ ") ="+point.distance(pred.formeTri.transfoMatrix.mult(pred.formeTri.points.get(joint.pointsIdx.get(numPointToCheckAeff))))+" > 0.0001");
				numPointToCheckAeff ++;
			}
		}
		if(pred.formeTri.joint instanceof JointPonctuel){
			JointPonctuel joint = (JointPonctuel) pred.formeTri.joint;
			System.out.println("Joint ponctuelP has : " + joint.pointWRotation + ".distance"+joint.pointLRotation+"("
					+pred.formeTri.transfoMatrix.mult(joint.pointLRotation)
					+ ") ="+joint.pointWRotation.distance(pred.formeTri.transfoMatrix.mult(joint.pointLRotation))+" > 0.0001");
		}

		if(pred.formePoint.joint instanceof JointPose){
			JointPose joint = (JointPose) pred.formePoint.joint;
			System.out.println("Check Coll forme point "+pred.formePoint+" : ");
			int numPointToCheckAeff = 0;
			while (numPointToCheckAeff < joint.points.size()) {
					Vector3f point = joint.points.get(numPointToCheckAeff);
					System.out.println("Joint pose already has : " + point + ".distance("
							+ pred.formePoint.transfoMatrix.mult(pred.formePoint.points.get(joint.pointsIdx.get(numPointToCheckAeff))) 
							+ ") ="+point.distance(pred.formePoint.transfoMatrix.mult(pred.formePoint.points.get(joint.pointsIdx.get(numPointToCheckAeff))))+" > 0.0001");
				numPointToCheckAeff ++;
			}
		}
		if(pred.formePoint.joint instanceof JointPonctuel){
			JointPonctuel joint = (JointPonctuel) pred.formePoint.joint;
			System.out.println("Joint ponctuelT has : " + joint.pointWRotation + ".distance"+joint.pointLRotation+"("
					+pred.formePoint.transfoMatrix.mult(joint.pointLRotation)
					+ ") ="+joint.pointWRotation.distance(pred.formePoint.transfoMatrix.mult(joint.pointLRotation))+" > 0.0001");
		}

		// move to collision point

		// check what object to move
		// Vector3f vitesseP2T =
		// pred.formeTri.vitesse.negate().addLocal(pred.formePoint.vitesse);
		Vector3f vitesseAPC = pred.formePoint.vangulaire.cross(pred.localP);
		Vector3f vitesseATC = pred.formeTri.vangulaire.cross(tempVect.set(pred.localTA).addLocal(pred.localTB)
				.addLocal(pred.localTC).divideLocal(3));

		Vector3f vitesseAP = new Vector3f(vitesseAPC);
		Vector3f vitesseAT = new Vector3f(vitesseATC);
		// TODOAFTER: if you remove the angular velocity, maybe instead
		// remove all interpolation here
		// and use the fined-grained interpolation.
		// remove angular interpolation if it's too high (more than 0.5
		// rad/s).
		// i'm not confident in this formulae, but it's not very
		// important.
		if (vitesseAP.lengthSquared() > FastMath.PI * .25f) {
			vitesseAP.set(0, 0, 0);
		}
		if (vitesseAT.lengthSquared() > FastMath.PI * .25f) {
			vitesseAT.set(0, 0, 0);
		}

		float vitP = vitesseAP.add(pred.formePoint.vitesse).length();
		float vitT = vitesseAT.add(pred.formeTri.vitesse).length();
		
		///reduce vitesse for recalage
		vitP = 0;
		vitT = 0;

		float totalVit = vitesseAP.add(pred.formePoint.vitesse).subtractLocal(vitesseAT)
				.subtractLocal(pred.formeTri.vitesse).length();
		if (totalVit == 0)
			totalVit = 1;
		float percentVitP = vitP / totalVit;
		float percentVitT = vitT / totalVit;
		System.out.println("vitesseAP=" + vitesseAP + ", vitesseAT=" + vitesseAT);
		System.out.println("pred.formePoint.vitesse=" + pred.formePoint.vitesse + ", pred.formeTri.vitesse="
				+ pred.formeTri.vitesse);
		System.out
				.println("vitesseAP.add(pred.formePoint.vitesse).subtractLocal(vitesseAT).subtractLocal(pred.formeTri.vitesse).length()="
						+ vitesseAP.add(pred.formePoint.vitesse).subtractLocal(vitesseAT)
								.subtractLocal(pred.formeTri.vitesse).length());
		System.out.println("vitP=" + vitP + ", vitT=" + vitT + ", totalVit=" + totalVit);
		System.out.println("formeTri %vit : " + percentVitT);
		// percentVitP+percentVitT is >= 1
		
		// System.out.println("previous FormePoint : " +
		// pred.formePoint.position);
		System.out.println("previous PointPos : " + pred.formePoint.transfoMatrix.mult(pred.localP));
		// System.out.println("objectif FormePoint : " +
		// pred.formePoint.position.toVec3f().addLocal(pred.bestP.subtract(pred.worldP).mult(percentVitP)));
		if (percentVitP > 0) {
			float percentIntegrate = percentVitP * pred.bestP.distance(pred.worldP) / vitP;
			pred.formePoint.calculF.set(pred.formePoint.vitesse).multLocal(percentIntegrate);
			pred.formePoint.position.addLocal(pred.formePoint.calculF);
			Quaternion quaterAdd = new Quaternion().fromAngleAxis(pred.formePoint.vangulaire.length()
					* percentIntegrate, pred.formePoint.vangulaire);
			pred.formePoint.pangulaire.multLocal(quaterAdd);
			pred.formePoint.pangulaire.normalizeLocal();
			pred.formePoint.transfoMatrix.setTransform(
					pred.formePoint.position.toVec3fLocal(pred.formePoint.calculF), Vector3f.UNIT_XYZ,
					pred.formePoint.pangulaire.toRotationMatrix());
			// pred.formePoint.position.addLocal(pred.bestP.subtract(pred.worldP).mult(percentVitP));
			// pred.formePoint.transfoMatrix.setTransform(
			// pred.formePoint.position.toVec3fLocal(pred.formePoint.calculF),
			// Vector3f.UNIT_XYZ,
			// pred.formePoint.pangulaire.toRotationMatrix());
			pred.formePoint.joint.updatePosition(currentTime, (long)(dts*(double)percentVitP));
		}
		System.out.println("after FormePoint : " + pred.formePoint.position);
		System.out.println("after PointPos : " + pred.formePoint.transfoMatrix.mult(pred.localP));
		System.out.println("needed PointPos : " + pred.bestP);

		System.out.println("previous formeTri : " + pred.formeTri.position);
		System.out.println("objectif formeTri : "
				+ pred.formeTri.position.toVec3f()
						.addLocal(pred.bestP.subtract(pred.worldP).mult(-percentVitT)));

		// System.out.println("previous formeTriS : " + percentVitT);
		// System.out.println("previous pred.worldP.subtract(pred.bestP) : "
		// + pred.worldP.subtract(pred.bestP));
		// pred.formeTri.position.addLocal(vitesseAT.mult(percentVitT));
		System.out.println("previous computedPointOnTri : " + pred.bestP);
		Vector3f localBestPos = pred.formeTri.transfoMatrix.invert().mult(pred.bestP);
		if (percentVitT > 0) {
			float percentIntegrate = percentVitT * pred.bestP.distance(pred.worldP) / vitT;
			System.out.println("dist=" + pred.bestP.distance(pred.worldP) + ", percentIntegrate : "
					+ percentIntegrate);
			pred.formeTri.calculF.set(pred.formeTri.vitesse).multLocal(percentIntegrate);
			pred.formeTri.position.addLocal(pred.formeTri.calculF);
			Quaternion quaterAdd = new Quaternion().fromAngleAxis(pred.formeTri.vangulaire.length()
					* percentIntegrate, pred.formeTri.vangulaire);
			pred.formeTri.pangulaire.multLocal(quaterAdd);
			pred.formeTri.pangulaire.normalizeLocal();
			pred.formeTri.transfoMatrix.setTransform(
					pred.formeTri.position.toVec3fLocal(pred.formeTri.calculF), Vector3f.UNIT_XYZ,
					pred.formeTri.pangulaire.toRotationMatrix());
			// pred.formeTri.position.addLocal(pred.worldP.subtract(pred.bestP).mult(percentVitT));
			// pred.formeTri.transfoMatrix.setTransform(pred.formeTri.position.toVec3fLocal(pred.formeTri.calculF),
			// Vector3f.UNIT_XYZ,
			// pred.formeTri.pangulaire.toRotationMatrix());
			pred.formePoint.joint.updatePosition(currentTime, (long)(dts*(double)percentVitT));
		}
		System.out.println("after formeTri : " + pred.formeTri.position);
		System.out.println("after PointPos(tri) : " + pred.formePoint.transfoMatrix.mult(pred.localP));
		pred.formeTri.transfoMatrix.mult(localBestPos, pred.bestP);
		// System.out.println("computed needed PointPos(tri) : " +
		// pred.bestP);
		System.out.println("computed obj PointPos(tri) : " + pred.bestP);
		System.out.println("computed needed PointPos(triFromP) : "
				+ pred.formePoint.transfoMatrix.mult(pred.localP, new Vector3f()));

		// check if no other points are inside the two objects
		// if any, do ???
		// TODOAFTER

		if(pred.formeTri.joint instanceof JointPonctuel){
			JointPonctuel joint = (JointPonctuel) pred.formeTri.joint;
			System.out.println("2Joint ponctuel has : " + joint.pointWRotation + ".distance("
					+pred.formeTri.transfoMatrix.mult(joint.pointLRotation)
					+ ") ="+joint.pointWRotation.distance(pred.formeTri.transfoMatrix.mult(joint.pointLRotation))+" > 0.0001");
		}
		if(pred.formePoint.joint instanceof JointPonctuel){
			JointPonctuel joint = (JointPonctuel) pred.formePoint.joint;
			System.out.println("2Joint ponctuel has : " + joint.pointWRotation + ".distance("
					+pred.formePoint.transfoMatrix.mult(joint.pointLRotation)
					+ ") ="+joint.pointWRotation.distance(pred.formePoint.transfoMatrix.mult(joint.pointLRotation))+" > 0.0001");
		}
			

		// just check if on the same plane, or almost
		Vector3f tempA = new Vector3f();
		Vector3f tempB = new Vector3f();
		Vector3f tempC = new Vector3f();
		Vector3f tempP = new Vector3f();
		tempA = pred.formeTri.transfoMatrix.mult(pred.localTA, new Vector3f());
		tempB = pred.formeTri.transfoMatrix.mult(pred.localTB, new Vector3f());
		tempC = pred.formeTri.transfoMatrix.mult(pred.localTC, new Vector3f());
		tempP = pred.formePoint.transfoMatrix.mult(pred.localP, new Vector3f());
		Plane planTri = new Plane();
		planTri.setPlanePoints(tempA, tempB, tempC);
		System.out.println("Point previousPos : " + pred.worldP);
		System.out.println("Point computedPos : " + pred.bestP);
		System.out.println("Point newPos : " + tempP);
		System.out.println("Triangle previousPos: " + pred.worldTA + ", " + pred.worldTB + ", " + pred.worldTC);
		System.out.println("Triangle newPos: " + tempA + ", " + tempB + ", " + tempC);

//		Ray ray = new Ray();
//		ray.setOrigin(tempP);
//		ray.setDirection(pred.rayon.direction);
//		System.out.print("rayon touche: " + ray.intersects(tempA, tempB, tempC));
//		ray.intersectWhere(tempA, tempB, tempC, tempVect);
//		System.out.println(" @" + tempVect);
//		ray.setDirection(pred.rayon.direction.negate());
//		System.out.print("rayonN touche: " + ray.intersects(tempA, tempB, tempC));
//		ray.intersectWhere(tempA, tempB, tempC, tempVect);
//		System.out.println(" @" + tempVect);
//		System.out.println("pred.formePoint.position : " + pred.formePoint.position);

		// ok for linear only

		// now split the geometry (in the same plane)
		//TODO?: check if an existing point is already here, then use it
		//	 c'est mieux quand on a un système oscilant ou coincé mais mouvant  ?
		createNewPoint(pred.formeTri, pred.triIdx, pred.bestP);
		int idxNewPointInTri = pred.formeTri.points.size() -1;

		// now it's sufficiently near for low-velocity objects
		// but it's not enough for high-velocity (and with angular
		// interpolation removed).
		Vector3f previousBestP = new Vector3f();
		//TODO: sometimes, it fail to integrate with a sufficient precision.
		//TODO: why 2 boucle? use only 1! => do the rotation/translation to the plane!
		System.out.println("tempP.distance(pred.bestP) = "+tempP.distance(pred.bestP));
		while(tempP.distance(pred.bestP)>0.00005f && !previousBestP.equals(pred.bestP)){
			previousBestP.set(pred.bestP);
			System.out.println("pred.formePoint.position : " + pred.formePoint.position);
			int possibilityJointP = pred.formePoint.joint.degreeOfLiberty();
			int possibilityJointT = pred.formeTri.joint.degreeOfLiberty();
			if (possibilityJointP > possibilityJointT) {
				pred.formePoint.joint.gotoCollision(pred.pointIdx, planTri);
			} else if (possibilityJointP > possibilityJointT) {
				pred.formeTri.joint.gotoCollision(pred.localTA, pred.localTB, pred.localTC, tempP);
			} else {
				// use the one who move faster
				if (vitesseAPC.addLocal(pred.formePoint.vitesse).length() >= vitesseATC.addLocal(
						pred.formeTri.vitesse).length()) {
					pred.formePoint.joint.gotoCollision(pred.pointIdx, planTri);
				} else {
					pred.formeTri.joint.gotoCollision(pred.localTA, pred.localTB, pred.localTC,  tempP);
				}
			}
			System.out.println("pred.formePoint.position AFTER : " + pred.formePoint.position);
			System.out.println("Point newPos5 : " + tempP);
			// now place tri at the right place (with thing as long as 200m
			// in rotation, it can create error like 0.8m)
			//now, with gotoCollision done, the error is below 0.1 mm
			Vector3f localTriP = pred.formeTri.points.get(idxNewPointInTri);
			tempA = pred.formeTri.transfoMatrix.mult(pred.localTA, new Vector3f());
			tempB = pred.formeTri.transfoMatrix.mult(pred.localTB, new Vector3f());
			tempC = pred.formeTri.transfoMatrix.mult(pred.localTC, new Vector3f());
			tempP = pred.formePoint.transfoMatrix.mult(pred.localP, new Vector3f());
			System.out.println("Point newPos6 : " + tempP);
			planTri.setPlanePoints(tempA, tempB, tempC);
			pred.bestP = planTri.getClosestPoint(tempP);
			System.out.println("set tri point from "+pred.formeTri.transfoMatrix.mult(localTriP)
					+" to "+pred.bestP+" near "+tempP+" => dist="+tempP.distance(pred.bestP)+" =?= "
					+ planTri.pseudoDistance(tempP));
			localTriP.set(pred.formeTri.transfoMatrix.invert().mult(pred.bestP));
		
		};

		//TODO
		//if one is inside the other and we failed to recover
		// => revert to previous good pos and integrate eux deux with small steps
		// instead of a big for eux deux + small ones for un seul.
		
		
		// create the link between the two objects
		// JointPonctuel joint = new JointPonctuel();
		// joint.f1 = pred.formeTri;
		// joint.idxPointf1 = idxNewPointInTri;
		// joint.f2 = pred.formePoint;
		// joint.idxPointf2 = pred.pointIdx;
		// joint.point = pred.bestP;
		// pred.formePoint.joint.add(joint);
		// pred.formeTri.joint.add(joint);
		System.out.println("CU add a new colision point (P): "+pred.formePoint+" : "+tempP+" == "
		+pred.formePoint.transfoMatrix.mult(pred.formePoint.points.get(pred.pointIdx)));
		pred.formePoint.joint.addCollisionPoint(tempP, pred.pointIdx, 
				pred.formeTri,  idxNewPointInTri);
		System.out.println("CU add a new colision point (T): "+pred.formeTri+" : "+pred.bestP+" == "
		+pred.formeTri.transfoMatrix.mult(pred.formeTri.points.get(idxNewPointInTri)));
		pred.formeTri.joint.addCollisionPoint(pred.bestP, idxNewPointInTri,
				pred.formePoint, pred.pointIdx);

		// TODO add energy->spedd from the current velocity ?
		System.out.println("COLLISON => REMOVE ALL SPEED");
		pred.formePoint.vangulaire.set(0, 0, 0);
		pred.formePoint.vitesse.set(0, 0, 0);
		pred.formePoint.acceleration.set(0, 0, 0);
		pred.formePoint.aangulaire.set(0, 0, 0);
		pred.formeTri.vangulaire.set(0, 0, 0);
		pred.formeTri.vitesse.set(0, 0, 0);
		pred.formeTri.acceleration.set(0, 0, 0);
		pred.formeTri.aangulaire.set(0, 0, 0);

		// clear collision, need to re-init all of them now.
		pred.formePoint.collideAt.clear();
		pred.formeTri.collideAt.clear();
		predictions.remove(pred);
		// joint.computeJointForce(pred.formePoint);
		// joint.computeJointForce(pred.formeTri);


		if(pred.formeTri.joint instanceof JointPonctuel){
			JointPonctuel joint = (JointPonctuel) pred.formeTri.joint;
			System.out.println("CU 3Joint ponctuel has (T): " + joint.pointWRotation + ".distance("
					+pred.formeTri.transfoMatrix.mult(joint.pointLRotation)
					+ ") ="+joint.pointWRotation.distance(pred.formeTri.transfoMatrix.mult(joint.pointLRotation))+" > 0.0001");
		}
		if(pred.formePoint.joint instanceof JointPonctuel){
			JointPonctuel joint = (JointPonctuel) pred.formePoint.joint;
			System.out.println("CU 3Joint ponctuel has (P): " + joint.pointWRotation + ".distance("
					+pred.formePoint.transfoMatrix.mult(joint.pointLRotation)
					+ ") ="+joint.pointWRotation.distance(pred.formePoint.transfoMatrix.mult(joint.pointLRotation))+" > 0.0001");
		}
		

	}
	
	

	private Vector3f createNewPoint(Forme formeTri, int triIdx, Vector3f newPoint) {
		System.out.println("create new point @" + newPoint);
		int idxNewP = formeTri.points.size();
		formeTri.addPoint(formeTri.transfoMatrix.invert().mult(newPoint));
		Triangle triInit = formeTri.triangles.get(triIdx);
		Triangle tri2 = new Triangle(formeTri, triInit.b, triInit.c, idxNewP);
		Triangle tri3 = new Triangle(formeTri, triInit.c, triInit.a, idxNewP);
		triInit.c = idxNewP;
		formeTri.triangles.add(tri2);
		formeTri.triangles.add(tri3);

		// make them face the good side (can be removed, i think
		Vector3f tempVect = new Vector3f(0, 0, 0);
		Triangle[] triangles = new Triangle[] { triInit, tri2, tri3 };

		Plane plan = new Plane();
		for (Triangle tri : triangles) {
			plan.setPlanePoints((formeTri.points.get(tri.a)), (formeTri.points.get(tri.b)),
					(formeTri.points.get(tri.c))); // create plane
			//set normal for the new point
			formeTri.normales.get(formeTri.normales.size()-1).set(plan.getNormal());
			//order the points in the right direction
			if (plan.whichSide(tempVect) == Side.Positive) {
				System.err.println("Forme " + this + " : wrong side for newly created triangle " + tri.a + tri.b
						+ tri.c);
				int temp = tri.b;
				tri.b = tri.a;
				tri.a = temp;
			}
		}

		return newPoint;
	}

	private void checkPredictionRealization(CollisionPrediction pred, long currentTime, long dtms) {
		if(pred.pointIdx < 0) return;
		Vector3f tempVect = new Vector3f();
		float dts = dtms * 0.001f;
		// TODOAFTER: que la fonction précédente cache ces valeurs dans l'objet
		// collPred... ca evite de les recalculer ici.
		// Vector3f tempA = new Vector3f();
		// Vector3f tempB = new Vector3f();
		// Vector3f tempC = new Vector3f();
		// Vector3f tempP = new Vector3f();
		// Vector3f triA = new Vector3f();
		// Vector3f triB = new Vector3f();
		// Vector3f triC = new Vector3f();
		// Vector3f pointP = new Vector3f();
		// triA =
		// pred.formeTri.points.get(pred.formeTri.triangles.get(pred.triIdx).a);
		// triB =
		// pred.formeTri.points.get(pred.formeTri.triangles.get(pred.triIdx).b);
		// triC =
		// pred.formeTri.points.get(pred.formeTri.triangles.get(pred.triIdx).c);
		// pred.formeTri.transfoMatrix.mult(triA, tempA);
		// pred.formeTri.transfoMatrix.mult(triB, tempB);
		// pred.formeTri.transfoMatrix.mult(triC, tempC);
		// pointP = pred.formePoint.points.get(pred.pointIdx);
		// pred.formePoint.transfoMatrix.mult(pointP, tempP);
		// compute velocity
		Vector3f vitesseP2T = pred.formeTri.vitesse.negate().addLocal(pred.formePoint.vitesse);
		Vector3f vitesseAP = pred.formePoint.vangulaire.cross(pred.worldP.subtract(pred.formePoint.position.toVec3f()));
		Vector3f vitesseATN = pred.formeTri.vangulaire.cross(
				tempVect.set(pred.worldTA).addLocal(pred.worldTB).addLocal(pred.worldTC).divideLocal(3)
				.subtractLocal(pred.formePoint.position.toVec3f())).negateLocal();

		System.out.println("vitesseP2T="+vitesseP2T);
		System.out.println("vitesseATN="+vitesseATN+", rotVector="+pred.formeTri.vangulaire);
		System.out.println("vitesseAP="+vitesseAP+", rotVector="+pred.formePoint.vangulaire);
		System.out.println("pred.formePoint.position="+pred.formePoint.position);
		System.out.println("pred.formeTri.position="+pred.formeTri.position);
		Vector3f velocityTot = vitesseP2T.add(vitesseAP).subtractLocal(vitesseATN);

		// this method work only if rayon as a normal vector
		// System.out.println("CHECK REA : "+pred.rayon.intersects(pred.worldTA,
		// pred.worldTB, pred.worldTC)+
		// " <? "+ (velocityTot.length() * dt));
		System.out.println("tri vit = " + pred.formeTri.vitesse + " length = " + pred.formeTri.vitesse.length());
		System.out.println("pointVit = " + pred.formePoint.vitesse + " length = " + pred.formePoint.vitesse.length());
		System.out.println("point-tri vit = " + vitesseP2T + " length = " + vitesseP2T.length());
		System.out.println("tri Avit = " + pred.formePoint.vangulaire.cross(pred.localP) + " length = "
				+ pred.formePoint.vangulaire.cross(pred.localP).length());
		System.out.println("point Avit = "
				+ pred.formeTri.vangulaire.cross(tempVect.set(pred.localTA).addLocal(pred.localTB)
						.addLocal(pred.localTC).divideLocal(3))
				+ " length = "
				+ pred.formeTri.vangulaire.cross(tempVect.set(pred.localTA).addLocal(pred.localTB)
						.addLocal(pred.localTC).divideLocal(3)));
		System.out.println("totalvit = "
				+ pred.formeTri.vitesse.negate().addLocal(vitesseATN).addLocal(pred.formePoint.vitesse)
						.addLocal(vitesseAP)
				+ ", length = "
				+ pred.formeTri.vitesse.negate().addLocal(vitesseATN).addLocal(pred.formePoint.vitesse)
						.addLocal(vitesseAP).length());

		System.out.println("CHECK REA : " + pred.bestP.distance(pred.worldP) + " <? " + (velocityTot.length() * dts));
		System.out.println("CHECK REA2 : " + pred.bestP.subtract(pred.worldP).dot(velocityTot)+" > 0");
		System.out.println("bestP = " + pred.bestP);
		System.out.println("worldP = " + pred.worldP);
		System.out.println("velocityTot = " + velocityTot);
		System.out.println("dts = " + dts);
		System.out.println("velocityTotLength*dts = " + velocityTot.length() * dts);
		System.out.println("pred.bestP.subtract(pred.worldP) = " + pred.bestP.subtract(pred.worldP));
		System.out.println("velocityTot = " + velocityTot);
		System.out.println("dot = " + pred.bestP.subtract(pred.worldP).dot(velocityTot));
		

//		Vector3f m = pred.worldP;//formePoint.transfoMatrix.mult(point, null);
//		Vector3f om = m.subtract(pred.formePoint.position.toVec3f());
//		Vector3f rotVect = pred.formePoint.vangulaire.mult(1).crossLocal(om);
//		Vector3f n = rotVect;
//		n.addLocal(pred.formePoint.vitesse.mult(1));
//		System.out.println("pred.worldP = " + pred.worldP);
//		System.out.println("pred.formePoint.position.toVec3f() = " + pred.formePoint.position.toVec3f());
//		System.out.println("om = " + om);
//		System.out.println("rotVect = " + rotVect);
//		System.out.println("pred.formePoint.vitesse = " + pred.formePoint.vitesse);
//		System.out.println("velocity2 = " + n);
//		System.out.println("dot2 = " + pred.bestP.subtract(pred.worldP).dot(n));
//
//		m = pred.worldTA.add(pred.worldTB).addLocal(pred.worldTC).divideLocal(3);//formePoint.transfoMatrix.mult(point, null);
//		om = m.subtract(pred.formeTri.position.toVec3f());
//		rotVect = pred.formeTri.vangulaire.mult(1).crossLocal(om);
//		n = rotVect;
//		n.addLocal(pred.formeTri.vitesse.mult(1));
//		System.out.println("pred.worldTP = " + m );
//		System.out.println("pred.formeTri.position.toVec3f() = " + pred.formeTri.position.toVec3f());
//		System.out.println("om = " + om);
//		System.out.println("rotVect = " + rotVect);
//		System.out.println("pred.formeTri.vitesse = " + pred.formeTri.vitesse);
//		System.out.println("velocity2 = " + n);
//		System.out.println("dot2 = " + pred.bestP.subtract(pred.worldP).dot(n));
		
		
		

		if ( (pred.bestP.subtract(pred.worldP).dot(velocityTot) > 0 || pred.bestP.distance(pred.worldP) < 0.00001)
				&& pred.bestP.distance(pred.worldP) < velocityTot.length() * dts * 1.05f /* hate rounding errors */) {
			// it collide!
			System.out.println("==================COLLIDE============================");

			// TODO: rotate a small amount to integrate the rotation a bit
			// further
			// but do it in update position, not here

			// move to approxim pos (witout integrating rotation 4now)
			// pred.formePoint.collideAt.put(pred.formeTri, pred);
			// pred.formeTri.collideAt.put(pred.formePoint, pred);
			pred.formePoint.collideAt.add(pred);
			pred.formeTri.collideAt.add(pred);
			pred.formePoint.predictions.remove(pred.formeTri);
			pred.formeTri.predictions.remove(pred.formePoint);
			System.out.println("collide @ " + pred.bestP + " == " + pred.worldP);
		}
		System.out.println("pred.formePoint.position="+pred.formePoint.position);
		System.out.println("pred.formeTri.position="+pred.formeTri.position);

	}

	private void updatePrediction(CollisionPrediction pred, long currentTime) {

		// TODO: update only current prediction if rotVel is not too high par
		// rapport à la distance.

		initPrediction(pred, currentTime);

	}

	private void initPrediction(CollisionPrediction pred, long currentTime) {
		Forme f1 = pred.formePoint;
		Forme f2 = pred.formeTri;
		Vector3f triA;
		Vector3f triB;
		Vector3f triC;
		Vector3f tempA = new Vector3f();
		Vector3f tempB = new Vector3f();
		Vector3f tempC = new Vector3f();
		Vector3f tempP = new Vector3f();
		Vector3f tempCalc = new Vector3f();
		Vector3f rotVelTri = new Vector3f();
		Vector3f computeVelocity = new Vector3f();
		Vector3f bestP = new Vector3f();
		Forme bestPointeur = null;
		Forme bestFormeTri = null;
		float distMin = 10000000;
		int bestIdxTri = -1;
		int bestIdxPoint = -1;
		Ray ray = new Ray();

		// ???
		HashSet<Integer> badIdxF1 = new HashSet<>(f1.joint.getIdx());
		// new ArrayList<>();
		// for( JointPonctuel j : f1.joint){
		// if(j.f1 == f1){
		// badIdxF1.add(j.idxPointf1);
		// }else{
		// badIdxF1.add(j.idxPointf2);
		// }
		// }
		HashSet<Integer> badIdxF2 = new HashSet<>(f2.joint.getIdx());
		// for( JointPonctuel j : f2.joint){
		// if(j.f1 == f2){
		// badIdxF2.add(j.idxPointf1);
		// }else{
		// badIdxF2.add(j.idxPointf2);
		// }
		// }
		System.out.println("badIdxF1 : " + badIdxF1);
		System.out.println("badIdxF2 : " + badIdxF2);
		// / END ???

		// create new dir
		Vector3f vitesseF1 = f2.vitesse.negate().addLocal(f1.vitesse);
		// ray.setDirection(vitesseF1);
		System.out.println("--------------- rayCollision ---------- " + vitesseF1);
		System.out.println("between " + f1 + " and " + f2);
		Forme pointeur = f1;
		Forme formeTri = f2;
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
			tempCalc.set(tempA).addLocal(tempB).addLocal(tempC).divideLocal(3).subtractLocal(formeTri.position.toVec3f());
			rotVelTri.set(formeTri.vangulaire).crossLocal(tempCalc).negateLocal();

			for (int idxPoint = 0; idxPoint < pointeur.points.size(); idxPoint++) {
				if (badIdxF1.contains(idxPoint))
					continue;
				Vector3f pointLocalPos = pointeur.points.get(idxPoint);
				pointeur.transfoMatrix.mult(pointLocalPos, tempP);
//				 System.out.println("Change point: " +idxPoint+
//				 "@"+tempP+" ("+pointLocalPos+")");
				ray.setOrigin(tempP);
				// add rotationalForce
				computeVelocity.set(pointeur.vangulaire).crossLocal(tempP.subtract(pointeur.position.toVec3f()));
//				System.out.println("vitesseF1 = "+vitesseF1+", rotVelo= "+computeVelocity);
				computeVelocity.addLocal(vitesseF1).addLocal(rotVelTri);
				if (computeVelocity.lengthSquared() == 0)
					continue;
				ray.setDirection(computeVelocity.normalizeLocal());
//				 System.out.println("1test on tri " +idxTri+
//				 " (p "+idxPoint+") with ray "+ray+" "+ray.intersectWhere(tempA, tempB, tempC, tempCalc));
//				 System.out.println("tri = "+tempA+ " "+tempB+" "+ tempC);
//				 System.out.println("intersection @" +tempCalc+" dist="+ray.intersects(tempA, tempB, tempC));

				if (ray.intersectWhere(tempA, tempB, tempC, tempCalc)) {
					float dist = (tempCalc.distance(tempP));
					if (dist < distMin) {
						System.out.println("1find a futur intersec on " + tempCalc + "@"
								+ (tempCalc.distance(tempP) + " in tri " + idxTri));
						System.out.println("from " + ray.getOrigin() + " (p " + idxPoint + ") with dir "
								+ ray.getDirection());
						
						System.out.println("vitesseF1 = "+vitesseF1+", rotVelo= "+computeVelocity);
						 System.out.println("1test on tri " +idxTri+
						 " (p "+idxPoint+") with ray "+ray+" "+ray.intersectWhere(tempA, tempB, tempC, tempCalc));
						 System.out.println("tri = "+tempA+ " "+tempB+" "+ tempC);
						 System.out.println("intersection @" +tempCalc+" dist="+ray.intersects(tempA, tempB, tempC));
						// System.out.println("vlinear = " + vitesseF1);
						// System.out.println("vAngulTri = " + rotVelTri);
						// System.out.println("vAngulP = "
						// +
						// computeVelocity.set(pointeur.vangulaire).crossLocal(pointLocalPos.normalize()));
						distMin = dist;
						bestP.set(tempCalc);
						bestPointeur = pointeur;
						bestFormeTri = formeTri;
						bestIdxTri = idxTri;
						bestIdxPoint = idxPoint;
						pred.localTA = triA;
						pred.localTB = triB;
						pred.localTC = triC;
						System.out.println("set localP @ " + pointLocalPos);
						pred.localP = pointLocalPos;
						pred.worldTA.set(tempA);
						pred.worldTB.set(tempB);
						pred.worldTC.set(tempC);
						pred.worldP.set(tempP);
						pred.rayon.set(ray);
					}
				}
			}
		}

		System.out.println("--------------- rayCollision 2 ---------- " + vitesseF1);
		System.out.println("between " + f2 + " and " + f1);
		// change forme
		// ray.setDirection(vitesseF1.negate());
		vitesseF1.negateLocal();
		pointeur = f2;
		formeTri = f1;
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
			tempCalc.set(tempA).addLocal(tempB).addLocal(tempC).divideLocal(3).subtractLocal(formeTri.position.toVec3f());
			rotVelTri.set(formeTri.vangulaire).crossLocal(tempCalc).negateLocal();

			for (int idxPoint = 0; idxPoint < pointeur.points.size(); idxPoint++) {
				if (badIdxF2.contains(idxPoint))
					continue;
				Vector3f pointLocalPos = pointeur.points.get(idxPoint);
				pointeur.transfoMatrix.mult(pointLocalPos, tempP);
				ray.setOrigin(tempP);
				// System.out.println("Change point: " +idxPoint+
				// "@"+tempP+" ("+pointLocalPos+")");

				// add rotationalForce
				computeVelocity.set(pointeur.vangulaire).crossLocal(tempP.subtract(pointeur.position.toVec3f()));
				computeVelocity.addLocal(vitesseF1).addLocal(rotVelTri);
				if (computeVelocity.lengthSquared() == 0)
					continue;
				ray.setDirection(computeVelocity.normalizeLocal());
				// System.out.println("2test on tri " +idxTri+
				// " (p "+idxPoint+") with ray "+ray);

				if (ray.intersectWhere(tempA, tempB, tempC, tempCalc)) {
					float dist = (tempCalc.distance(tempP));
					// System.out.println("2find a possible futur intersec on "
					// + tempCalc + "@"
					// + (tempCalc.distance(tempP) + " in tri " + idxTri));
					// System.out.println("from " + ray.getOrigin() +
					// " (p "+idxPoint+")  with dir " + ray.getDirection());
					if (dist < distMin) {
						System.out.println("2find a futur intersec on " + tempCalc + "@"
								+ (tempCalc.distance(tempP) + " in tri " + idxTri));
						System.out.println("from " + ray.getOrigin() + " (p " + idxPoint + ") with dir "
								+ ray.getDirection());

						System.out.println("vitesseF1 = "+vitesseF1+", rotVelo= "+computeVelocity+" rot vector = "+pointeur.vangulaire);
						 System.out.println("1test on tri " +idxTri+
						 " (p "+idxPoint+") with ray "+ray+" "+ray.intersectWhere(tempA, tempB, tempC, tempCalc));
						 System.out.println("tri = "+tempA+ " "+tempB+" "+ tempC);
						 System.out.println("intersection @" +tempCalc+" dist="+ray.intersects(tempA, tempB, tempC));
						distMin = dist;
						bestP.set(tempCalc);
						bestPointeur = pointeur;
						bestFormeTri = formeTri;
						bestIdxTri = idxTri;
						bestIdxPoint = idxPoint;
						pred.localTA = triA;
						pred.localTB = triB;
						pred.localTC = triC;
						pred.localP = pointLocalPos;
						System.out.println("set localP @ " + pointLocalPos);
						pred.worldTA.set(tempA);
						pred.worldTB.set(tempB);
						pred.worldTC.set(tempC);
						pred.worldP.set(tempP);
						pred.rayon.set(ray);
					}
				}
			}
		}
		System.out.println("--------------- END rayCollision ---------- ");
		if (bestPointeur != null) {
			pred.init = true;
			pred.formePoint = bestPointeur;
			pred.formeTri = bestFormeTri;
			pred.pointIdx = bestIdxPoint;
			pred.triIdx = bestIdxTri;
			pred.bestP = bestP;
			// compute time
			pred.moment = currentTime + (long) (distMin / vitesseF1.length());

		} // else : no collision possible, check some time later
			// else, add some time before redo the init? (dangerous!)
		else{

//			pred.formePoint = bestPointeur;
//			pred.formeTri = bestFormeTri;
			pred.pointIdx = -1;
			pred.triIdx = -1;
			pred.bestP = null;
			pred.moment = 0;
		}

		//TODO: triangle-triangle collision avec les arretes, sans points.
		
		
	}

	//
	// private void initPrediction(CollisionPrediction pred) {
	// Vector3f calcul = new Vector3f();
	// //note: formePoint & formeTri are interchangeable
	// //check with nearly points
	// Vector3f vitesseP = pred.formePoint.vitesse;
	// Vector3f vitesseT = pred.formeTri.vitesse;
	//
	// if(vitesseP.lengthSquared() + vitesseT.lengthSquared() == 0){
	// //can't init: they don't move: do nothing.
	// return;
	// }
	//
	// Vector3f vitesseP2T = vitesseT.negate().addLocal(vitesseP);
	// Vector3f farP = new
	// Vector3f(pred.formePoint.getMostFarAwayPoint(vitesseP2T, calcul));
	// Vector3f farT = new
	// Vector3f(pred.formePoint.getMostFarAwayPoint(vitesseP2T.negateLocal(),
	// calcul));
	//
	// //check with a plane => maybe use a direct calcul
	// Plane plan = new Plane();
	// plan.setOriginNormal(farP, vitesseP2T.negateLocal());
	//
	// System.out.println("plan: "+plan);
	// Side s = plan.whichSide(farT);
	//
	// if(s == Side.Positive){
	// //not enough near : don't init now.
	// return;
	// }
	//
	// //it's near ... peraps? and rotation? errrf?....
	// }

	// ACtuellement, uniquement translation, TODO: ajouter le rotation sur la
	// pointe (au moins)
	// static public CollisionPrediction predictRayCollision(Forme f1, Forme f2,
	// long currentTime){
	// Vector3f tempA = new Vector3f();
	// Vector3f tempB = new Vector3f();
	// Vector3f tempC = new Vector3f();
	// Vector3f tempP = new Vector3f();
	// Vector3f tempCalc = new Vector3f();
	// Vector3f bestP = new Vector3f();
	// Forme bestPointeur = null;
	// Forme bestFormeTri = null;
	// float distMin = 10000000;
	// int bestIdxTri = -1;
	// int bestIdxPoint = -1;
	// Ray ray = new Ray();
	// //create new dir
	// Vector3f vitesseF1 = f2.vitesse.negate().addLocal(f1.vitesse);
	// ray.setDirection(vitesseF1);
	// System.out.println("--------------- rayCollision ---------- "+vitesseF1);
	// Forme pointeur = f1;
	// Forme formeTri = f2;
	// //check pointe from f1
	// for(int idxTri =0; idxTri < formeTri.points.size(); idxTri++){
	// Triangle tri = formeTri.triangles.get(idxTri);
	// //create the plane for this triangle
	// formeTri.transfoMatrix.mult(formeTri.points.get(tri.a), tempA);
	// formeTri.transfoMatrix.mult(formeTri.points.get(tri.b), tempB);
	// formeTri.transfoMatrix.mult(formeTri.points.get(tri.c), tempC);
	// for(int idxPoint =0; idxPoint < pointeur.points.size(); idxPoint++){
	// pointeur.transfoMatrix.mult(pointeur.points.get(idxPoint), tempP);
	// ray.setOrigin(tempP);
	//
	// if(ray.intersectWhere(tempA, tempB, tempC, tempCalc)){
	// float dist = (tempCalc.distance(tempP));
	// if(dist < distMin){
	// System.out.println("find a futur intersec on "+tempCalc+
	// "@"+(tempCalc.distance(tempP)+" in tri "+idxTri));
	// System.out.println("from "+ray.getOrigin()+ " with dir " +
	// ray.getDirection());
	// distMin = dist;
	// bestP.set(tempCalc);
	// bestPointeur = pointeur;
	// bestFormeTri = formeTri;
	// bestIdxTri = idxTri;
	// bestIdxPoint = idxPoint;
	// }
	// }
	// }
	// }
	// System.out.println("--------------- rayCollision2 ---------- "+vitesseF1.negate());
	// //check pointe from f2
	// ray.setDirection(vitesseF1.negate());
	// pointeur = f2;
	// formeTri = f1;
	// for(int idxTri =0; idxTri < formeTri.points.size(); idxTri++){
	// Triangle tri = formeTri.triangles.get(idxTri);
	// //create the plane for this triangle
	// formeTri.transfoMatrix.mult(formeTri.points.get(tri.a), tempA);
	// formeTri.transfoMatrix.mult(formeTri.points.get(tri.b), tempB);
	// formeTri.transfoMatrix.mult(formeTri.points.get(tri.c), tempC);
	// // System.out.println("testing tri "+tempA+ ", "+tempB+ ", "+tempC);
	// for(int idxPoint =0; idxPoint < pointeur.points.size(); idxPoint++){
	// pointeur.transfoMatrix.mult(pointeur.points.get(idxPoint), tempP);
	// ray.setOrigin(tempP);
	// //System.out.println("testing with orig: "+tempP+" : "+ray.intersects(tempA,
	// tempB, tempC));
	//
	// if(ray.intersectWhere(tempA, tempB, tempC, tempCalc)){
	// float dist = (tempCalc.distance(tempP));
	// if(dist < distMin){
	// System.out.println("find a futur intersec on "+tempCalc+
	// "@"+(tempCalc.distance(tempP)+" in tri "+idxTri));
	// System.out.println("from "+ray.getOrigin()+ " with dir " +
	// ray.getDirection());
	// distMin = dist;
	// bestP.set(tempCalc);
	// bestPointeur = pointeur;
	// bestFormeTri = formeTri;
	// bestIdxTri = idxTri;
	// bestIdxPoint = idxPoint;
	// }
	// }
	// }
	// }
	// System.out.println("--------------- END rayCollision ---------- ");
	// if(bestPointeur != null){
	// CollisionPrediction pred =new CollisionPrediction();
	// pred.formePoint = bestPointeur;
	// pred.formeTri = bestFormeTri;
	// pred.pointIdx = bestIdxPoint;
	// pred.triIdx = bestIdxTri;
	// //compute time
	// pred.moment = currentTime + (long)(distMin / vitesseF1.length());
	// return pred;
	// }
	// return null;
	// }

}
