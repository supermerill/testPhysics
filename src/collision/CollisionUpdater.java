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
import joint.JointPonctuel;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Matrix4f;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;
import com.jme3.math.Plane.Side;

import old.Forme;
import old.Forme.Triangle;
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
		// F: update list of future collision
		// ie check bb for mouving elems &elems with new forces
		for (Forme f1 : allFormes) {
			if (f1.physicUpdate) {
				for (Forme f2 : allFormes) {
					if (f1 == f2)
						continue;
					System.out.println("dist: " + f1.position.distance(f2.position) + " ? "
							+ (f1.roundBBRayon + f2.roundBBRayon));
					if (f1.position.distance(f2.position) < f1.roundBBRayon + f2.roundBBRayon) {
						System.out.println("possible collision ");
						// check if collision exist (and it's not a joint)
						CollisionPrediction collider = f1.predictions.get(f2);
						if (collider == null && !f1.joint.containsKey(f2)) {
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

	private void updatePos(Forme f, long currentTime, long dt) {
		// float dts = dt/1000f;
		// System.out.println("updatePos"+f.name);
		if (f.vitesse.lengthSquared() + f.vangulaire.lengthSquared() == 0) {
			// don't move => don't collide

			// TODO: check if this should done here or at other points

			// update linear speed
			f.calculF.set(f.lastAccel).multLocal(dt * 0.5f);
			f.vitesse.addLocal(f.calculF);
			f.calculF.set(f.acceleration).multLocal(dt * 0.5f);
			f.vitesse.addLocal(f.calculF);
			// System.out.println("now speed"+f.vitesse);

			// update angular speed
			f.calculF.set(f.lastAangulaire).multLocal(dt * 0.5f);
			f.vangulaire.addLocal(f.calculF);
			f.calculF.set(f.aangulaire).multLocal(dt * 0.5f);
			f.vangulaire.addLocal(f.calculF);
		} else {

			// check number of collision
			if (f.collideAt.size() == 0) {
				// free flight

				// move "normally" (integrate by half)
				// System.out.println("vitesse"+f.vitesse);
				// System.out.println("dt "+dt);
				// System.out.println("position"+f.position);
				//
				// System.out.println("dt²"+dt * dt);
				// System.out.println("dt/1000"+dt * 0.001);
				// System.out.println("dt*dt/1000"+dt *dt * 0.001);
				// System.out.println("(dt/1000)²"+dt *dt * 0.000001);
				// System.out.println("grav(dt/1000)²"+9.81*dt *dt * 0.000001);
				// System.out.println("grav(dt²/1000)"+0.00981*dt *dt * 0.001);
				// move linear
				f.calculF.set(f.vitesse).multLocal(dt);
				f.position.addLocal(f.calculF);
				f.calculF.set(f.lastAccel).multLocal(dt * dt);
				f.position.addLocal(f.calculF);
				// System.out.println("position "+f.position);

//				if (f.posAxeRot.lengthSquared() == 0) {
					// move angular
					// Matrix3f matriceRot = new Matrix3f(Matrix3f.IDENTITY);
					// matriceRot.multCACACPABONLocal(f.vangulaire);
					// Matrix4f matriceAdd = new Matrix4f();
					// matriceAdd.setTransform(f.calculF, Vector3f.UNIT_XYZ,
					// matriceRot);
					// f.transfoMatrix.addLocal(matriceAdd);
					// keep quaternion? => it reduce rounding error
					f.pangulaire.fromRotationMatrix(f.transfoMatrix.toRotationMatrix());
					// float angle = f.vangulaire.length();
					// if (angle > 0.0) // the formulas from the link
					// {
					// f.pangulaire.addLocal(new Quaternion(
					// f.vangulaire.x * FastMath.sin(angle / 2.0f) / angle,
					// f.vangulaire.y * FastMath.sin(angle / 2.0f) / angle,
					// f.vangulaire.z * FastMath.sin(angle / 2.0f) / angle,
					// FastMath.cos(angle / 2.0f)));
					// }
					Quaternion previousAngle = new Quaternion(f.pangulaire);
					Quaternion quaterAdd = new Quaternion().fromAngleAxis(f.vangulaire.length() * dt, f.vangulaire);
					f.pangulaire.multLocal(quaterAdd);
					f.pangulaire.normalizeLocal();
					
					
					//more precise!
					
//					f.pangulaireP.multLocal(new Quaterniond().fromAngleAxis(f.vangulaire.length() * dt, new Vector3d(f.vangulaire)));
//					f.pangulaireP.normalizeLocal();

//				if (f.posAxeRot.lengthSquared() != 0)  {
//					// move forme to rot pos
//					//MAYTODO: find a better way with rot mat 
//					//http://www.euclideanspace.com/maths/geometry/affine/aroundPoint/index.htm
//
//					Matrix4d newRot = new Matrix4d();
//					newRot.setTransform(Vector3d.ZERO, Vector3d.UNIT_XYZ,
//							new Quaterniond(quaterAdd).toRotationMatrix());
//					
//					Matrix4d preciseTrsf = new Matrix4d();
//					preciseTrsf.setTransform(f.position, Vector3d.UNIT_XYZ,
//							previousAngle.toRotationMatrix());
//					
//					Vector3d transl = new Vector3d(f.posAxeRot).mult(1);
////					new Matrix4d(f.transfoMatrix).multNormal(transl, transl);
//					preciseTrsf.multNormal(transl, transl);
//					newRot.m03 = transl.x - newRot.m00 * transl.x - newRot.m01 * transl.y - newRot.m02
//							* transl.z;
//					newRot.m13 = transl.y - newRot.m10 * transl.x - newRot.m11 * transl.y - newRot.m12
//							* transl.z;
//					newRot.m23 = transl.z - newRot.m20 * transl.x - newRot.m21 * transl.y - newRot.m22
//							* transl.z;
//					System.out.println("Translation : "+newRot.toTranslationVector());
//					
//					//f.transfoMatrix.translateVect(newRot.toTranslationVector());
////					System.out.println("previousPs: "+f.position);
//					f.position.addLocal(newRot.toTranslationVector());
////					System.out.println("correctedPs: "+f.position);
////					f.transfoMatrix.set(newRot.multLocal(f.transfoMatrix));
//					Vector3d lastPosPoint = preciseTrsf.mult(new Vector3d(f.points.get(f.points.size()-1)));
//					System.out.println("Point pos: "+preciseTrsf.mult(new Vector3d(f.points.get(f.points.size()-1))));
//					//as it's an integration =>notlinear, discrete (via dt), replace the point at the good place
//
//					preciseTrsf.setTransform(f.position, Vector3d.UNIT_XYZ,
//							new Quaterniond(f.pangulaire).toRotationMatrix());
//					
//					Vector3d newPosPoint = preciseTrsf.mult(new Vector3d(f.points.get(f.points.size()-1)));
//					f.position.addLocal(lastPosPoint.subtractLocal(newPosPoint));
//					
//				}
				if (f.posAxeRot.lengthSquared() != 0)  {
					// move forme to rot pos
					//MAYTODO: find a better way with rot mat 
					//http://www.euclideanspace.com/maths/geometry/affine/aroundPoint/index.htm

					Matrix4f newRot = new Matrix4f();
					newRot.setTransform(Vector3f.ZERO, Vector3f.UNIT_XYZ,
							quaterAdd.toRotationMatrix());
					
					Matrix4f preciseTrsf = new Matrix4f();
					preciseTrsf.setTransform(f.position.toVec3f(), Vector3f.UNIT_XYZ,
							previousAngle.toRotationMatrix());
					
					Vector3f transl = new Vector3f(f.posAxeRot).mult(1);
//					new Matrix4d(f.transfoMatrix).multNormal(transl, transl);
					preciseTrsf.multNormal(transl, transl);
					newRot.m03 = transl.x - newRot.m00 * transl.x - newRot.m01 * transl.y - newRot.m02
							* transl.z;
					newRot.m13 = transl.y - newRot.m10 * transl.x - newRot.m11 * transl.y - newRot.m12
							* transl.z;
					newRot.m23 = transl.z - newRot.m20 * transl.x - newRot.m21 * transl.y - newRot.m22
							* transl.z;
					System.out.println("Translation : "+newRot.toTranslationVector());
					
					//f.transfoMatrix.translateVect(newRot.toTranslationVector());
//					System.out.println("previousPs: "+f.position);
					f.position.addLocal(newRot.toTranslationVector());
//					System.out.println("correctedPs: "+f.position);
//					f.transfoMatrix.set(newRot.multLocal(f.transfoMatrix));
					Vector3f lastPosPoint = preciseTrsf.mult(new Vector3f(f.points.get(f.points.size()-1)));
					System.out.println("Point pos: "+preciseTrsf.mult(new Vector3f(f.points.get(f.points.size()-1))));
					//as it's an integration =>notlinear, discrete (via dt), replace the point at the good place

					preciseTrsf.setTransform(f.position.toVec3f(), Vector3f.UNIT_XYZ,
							f.pangulaire.toRotationMatrix());
					
					Vector3f newPosPoint = preciseTrsf.mult(new Vector3f(f.points.get(f.points.size()-1)));
					f.position.addLocal(lastPosPoint.subtractLocal(newPosPoint));
					
				}
				f.transfoMatrix.setTransform(f.position.toVec3fLocal(f.calculF), Vector3f.UNIT_XYZ,
					f.pangulaire.toRotationMatrix());
				
				// update linear speed
				f.calculF.set(f.lastAccel).multLocal(dt * 0.5f);
				f.vitesse.addLocal(f.calculF);
				f.calculF.set(f.acceleration).multLocal(dt * 0.5f);
				f.vitesse.addLocal(f.calculF);

				// update angular speed
				f.calculF.set(f.lastAangulaire).multLocal(dt * 0.5f);
				f.vangulaire.addLocal(f.calculF);
				f.calculF.set(f.aangulaire).multLocal(dt * 0.5f);
				f.vangulaire.addLocal(f.calculF);

			} else {
				// collision(s), use the nearset one.
				int idxCollision = 0;
				for (int i = 1; i < f.collideAt.size(); i++) {
					// TODO
				}

				CollisionPrediction pred = f.collideAt.get(idxCollision);
				System.out.println("Triangle forme pos @: " + pred.formeTri.position);
				System.out.println("Triangle localPos: " + pred.localTA + ", " + pred.localTB + ", " + pred.localTC);
				System.out.println("Triangle previousPos: " + pred.worldTA + ", " + pred.worldTB + ", " + pred.worldTC);
				Vector3f tempVect = new Vector3f();

				// TODO: integrate the angular composante a bit more
				// how many angular / linear composantes inside our velocity?
				// try it & recompute the choke point

				// move to collision point

				// check what object to move
				// Vector3f vitesseP2T =
				// pred.formeTri.vitesse.negate().addLocal(pred.formePoint.vitesse);
				Vector3f vitesseAP = pred.formePoint.vangulaire.cross(pred.localP);
				Vector3f vitesseAT = pred.formeTri.vangulaire.cross(tempVect.set(pred.localTA).addLocal(pred.localTB)
						.addLocal(pred.localTC).divideLocal(3));
				float vitP = vitesseAP.addLocal(pred.formePoint.vitesse).length();
				float vitT = vitesseAT.addLocal(pred.formeTri.vitesse).length();
				float totalVit = vitP + vitT;
				float percentVitP = vitP / totalVit;
				float percentVitT = vitT / totalVit;

//				 System.out.println("previous FormePoint : " +
//				 pred.formePoint.position);
//				 pred.formePoint.position.addLocal(vitesseAP.mult(percentVitP));
				pred.formePoint.position.addLocal(pred.bestP.subtract(pred.worldP).mult(percentVitP));
				pred.formePoint.transfoMatrix.setTransform(
						pred.formePoint.position.toVec3fLocal(pred.formePoint.calculF), Vector3f.UNIT_XYZ,
						pred.formePoint.pangulaire.toRotationMatrix());
//				 System.out.println("after FormePoint : " +
//				 pred.formePoint.position);

				 System.out.println("previous formeTri : " +
				 pred.formeTri.position);
				 pred.formeTri.position.addLocal(vitesseAT.mult(percentVitT));
				pred.formeTri.position.addLocal(pred.worldP.subtract(pred.bestP).mult(percentVitT));
				pred.formeTri.transfoMatrix.setTransform(pred.formeTri.position.toVec3fLocal(pred.formeTri.calculF),
						Vector3f.UNIT_XYZ, pred.formeTri.pangulaire.toRotationMatrix());

				 System.out.println("after formeTri : " +
				 pred.formeTri.position);

				// check if no other points are inside the two objects
				// if any,

				// just check if on the same plane, or almost
				Vector3f tempA = new Vector3f();
				Vector3f tempB = new Vector3f();
				Vector3f tempC = new Vector3f();
				Vector3f tempP = new Vector3f();
				tempA = pred.formeTri.transfoMatrix.mult(pred.localTA, new Vector3f());
				tempB = pred.formeTri.transfoMatrix.mult(pred.localTB, new Vector3f());
				tempC = pred.formeTri.transfoMatrix.mult(pred.localTC, new Vector3f());
				tempP = pred.formePoint.transfoMatrix.mult(pred.localP, new Vector3f());
				System.out.println("Point previousPos : " + pred.worldP);
				System.out.println("Point computedPos : " + pred.bestP);
				System.out.println("Point newPos : " + tempP);
				System.out.println("Triangle previousPos: " + pred.worldTA + ", " + pred.worldTB + ", " + pred.worldTC);
				System.out.println("Triangle newPos: " + tempA + ", " + tempB + ", " + tempC);

				Ray ray = new Ray();
				ray.setOrigin(tempP);
				ray.setDirection(pred.rayon.direction);
				System.out.print("rayon touche: " + ray.intersects(tempA, tempB, tempC));
				ray.intersectWhere(tempA, tempB, tempC, tempVect);
				System.out.println(" @" + tempVect);
				ray.setDirection(pred.rayon.direction.negate());
				System.out.print("rayonN touche: " + ray.intersects(tempA, tempB, tempC));
				ray.intersectWhere(tempA, tempB, tempC, tempVect);
				System.out.println(" @" + tempVect);

				// ok for linear only

				// now split the geometry
				createNewPoint(pred.formeTri, pred.triIdx, tempP);

				// create the link between the two objects
				JointPonctuel joint = new JointPonctuel();
				joint.f1 = pred.formeTri;
				joint.idxPointf1 = pred.formeTri.points.size() - 1;
				joint.f2 = pred.formePoint;
				joint.idxPointf2 = pred.pointIdx;
				joint.point = pred.bestP;
				pred.formePoint.joint.put(pred.formeTri, joint);
				pred.formeTri.joint.put(pred.formePoint, joint);

				// TODO add force from the current velocity ?

				// clear collision, need to re-init all of them now.
				pred.formePoint.collideAt.clear();
				pred.formeTri.collideAt.clear();
				predictions.remove(pred);
				joint.computeJointForce(pred.formePoint);
				joint.computeJointForce(pred.formeTri);

				// try {
				// Thread.sleep(1000000);
				// } catch (InterruptedException e) {
				// // TODO Auto-generated catch block
				// e.printStackTrace();
				// }

			}

		}

		// switch accel
		// Vector3f temp = f.lastAccel;
		f.lastAccel.set(f.acceleration);
		// f.acceleration = temp;
		// temp = f.lastAangulaire;
		f.lastAangulaire.set(f.aangulaire);
		// f.aangulaire = temp;

		// apply force on acceleration?
		// TODO
	}

	private Vector3f createNewPoint(Forme formeTri, int triIdx, Vector3f newPoint) {
		int idxNewP = formeTri.points.size();
		formeTri.points.add(formeTri.transfoMatrix.invert().mult(newPoint));
		Forme.Triangle triInit = formeTri.triangles.get(triIdx);
		Forme.Triangle tri2 = formeTri.new Triangle(triInit.b, triInit.c, idxNewP);
		Forme.Triangle tri3 = formeTri.new Triangle(triInit.c, triInit.a, idxNewP);
		triInit.c = idxNewP;
		formeTri.triangles.add(tri2);
		formeTri.triangles.add(tri3);

		// make them face the good side (can be removed, i think
		Vector3f tempVect = new Vector3f(0, 0, 0);
		Forme.Triangle[] triangles = new Forme.Triangle[] { triInit, tri2, tri3 };

		Plane plan = new Plane();
		for (Triangle tri : triangles) {
			plan.setPlanePoints((formeTri.points.get(tri.a)), (formeTri.points.get(tri.b)),
					(formeTri.points.get(tri.c))); // create plane
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

	private void checkPredictionRealization(CollisionPrediction pred, long currentTime, long dt) {
		Vector3f tempVect = new Vector3f();
		// TODO: que la fonction précédente cache ces valeurs dans l'objet
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
		Vector3f vitesseAP = pred.formePoint.vangulaire.cross(pred.localP);
		Vector3f vitesseATN = pred.formeTri.vangulaire.cross(
				tempVect.set(pred.localTA).addLocal(pred.localTB).addLocal(pred.localTC).divideLocal(3)).negateLocal();

		Vector3f velocityTot = vitesseP2T.add(vitesseAP).addLocal(vitesseATN);

		// this method work only if rayon as a normal vector
		// System.out.println("CHECK REA : "+pred.rayon.intersects(pred.worldTA,
		// pred.worldTB, pred.worldTC)+
		// " <? "+ (velocityTot.length() * dt));
		System.out.println("CHECK REA : " + pred.bestP.distance(pred.worldP) + " <? " + (velocityTot.length() * dt));

		if (pred.bestP.distance(pred.worldP) < velocityTot.length() * dt) {
			// it collide!

			// TODO: rotate a small amount to integrate the roation a bit
			// further
			// but do it in update position, not here

			// move to approxim pos (witout integrating rotation 4now)
			// pred.formePoint.collideAt.put(pred.formeTri, pred);
			// pred.formeTri.collideAt.put(pred.formePoint, pred);
			pred.formePoint.collideAt.add(pred);
			pred.formeTri.collideAt.add(pred);
		}

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
			tempCalc.set(triA).addLocal(triB).addLocal(triC).divideLocal(3);
			rotVelTri.set(formeTri.vangulaire).crossLocal(tempCalc);

			for (int idxPoint = 0; idxPoint < pointeur.points.size(); idxPoint++) {
				Vector3f pointLocalPos = pointeur.points.get(idxPoint);
				pointeur.transfoMatrix.mult(pointLocalPos, tempP);
				ray.setOrigin(tempP);
				// add rotationalForce
				computeVelocity.set(pointeur.vangulaire).crossLocal(pointLocalPos).negateLocal();
				computeVelocity.addLocal(vitesseF1).addLocal(rotVelTri);
				if (computeVelocity.lengthSquared() == 0)
					continue;
				ray.setDirection(computeVelocity.normalizeLocal());

				if (ray.intersectWhere(tempA, tempB, tempC, tempCalc)) {
					float dist = (tempCalc.distance(tempP));
					if (dist < distMin) {
						System.out.println("1find a futur intersec on " + tempCalc + "@"
								+ (tempCalc.distance(tempP) + " in tri " + idxTri));
						System.out.println("from " + ray.getOrigin() + " with dir " + ray.getDirection());
						System.out.println("vlinear = " + vitesseF1);
						System.out.println("vAngulTri = " + rotVelTri);
						System.out.println("vAngulP = "
								+ computeVelocity.set(pointeur.vangulaire).crossLocal(pointLocalPos).negateLocal());
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
			tempCalc.set(triA).addLocal(triB).addLocal(triC).divideLocal(3);
			rotVelTri.set(formeTri.vangulaire).crossLocal(tempCalc);

			for (int idxPoint = 0; idxPoint < pointeur.points.size(); idxPoint++) {
				Vector3f pointLocalPos = pointeur.points.get(idxPoint);
				pointeur.transfoMatrix.mult(pointLocalPos, tempP);
				ray.setOrigin(tempP);

				// add rotationalForce
				computeVelocity.set(pointeur.vangulaire).crossLocal(pointLocalPos).negateLocal();
				computeVelocity.addLocal(vitesseF1).addLocal(rotVelTri);
				if (computeVelocity.lengthSquared() == 0)
					continue;
				ray.setDirection(computeVelocity.normalizeLocal());

				if (ray.intersectWhere(tempA, tempB, tempC, tempCalc)) {
					float dist = (tempCalc.distance(tempP));
					System.out.println("2find a futur intersec on " + tempCalc + "@"
							+ (tempCalc.distance(tempP) + " in tri " + idxTri));
					System.out.println("from " + ray.getOrigin() + " with dir " + ray.getDirection());
					if (dist < distMin) {
						System.out.println("2find a BEST futur intersec on " + tempCalc + "@"
								+ (tempCalc.distance(tempP) + " in tri " + idxTri));
						System.out.println("from " + ray.getOrigin() + " with dir " + ray.getDirection());
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
