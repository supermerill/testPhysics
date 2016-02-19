package v2.joint;

import v2.Forme;
import jme3Double.PlaneD;
import jme3Double.Vector3d;


public class JointFreeFlight extends Joint {

	public JointFreeFlight(Forme f) {
		super(f);
	}

	@Override
	public void updatePosition(long instant, long dt) {
		//all of this is done in collision update
		// move linear
//		f.calculF.set(f.vitesse).multLocal(dt);
//		f.position.addLocal(f.calculF);
//		f.calculF.set(f.lastAccel).multLocal(dt * dt);
//		f.position.addLocal(f.calculF);
//
//		// move angular
//		// keep quaternion? => it reduce rounding error
//		f.pangulaire.fromRotationMatrix(f.transfoMatrix.toRotationMatrix());
//		// float angle = f.vangulaire.length();
//		// if (angle > 0.0) // the formulas from the link
//		// {
//		// f.pangulaire.addLocal(new Quaternion(
//		// f.vangulaire.x * FastMath.sin(angle / 2.0f) / angle,
//		// f.vangulaire.y * FastMath.sin(angle / 2.0f) / angle,
//		// f.vangulaire.z * FastMath.sin(angle / 2.0f) / angle,
//		// FastMath.cos(angle / 2.0f)));
//		// }
//		Quaternion quaterAdd = new Quaternion().fromAngleAxis(f.vangulaire.length() * dt, f.vangulaire);
//		f.pangulaire.multLocal(quaterAdd);
//		f.pangulaire.normalizeLocal();
//
//		f.transfoMatrix.setTransform(f.position.toVec3fLocal(f.calculF), Vector3d.UNIT_XYZ,
//				f.pangulaire.toRotationMatrix());
//
//		// update linear speed
//		f.calculF.set(f.lastAccel).multLocal(dt * 0.5f);
//		f.vitesse.addLocal(f.calculF);
//		f.calculF.set(f.acceleration).multLocal(dt * 0.5f);
//		f.vitesse.addLocal(f.calculF);
//
//		// update angular speed
//		f.calculF.set(f.lastAangulaire).multLocal(dt * 0.5f);
//		f.vangulaire.addLocal(f.calculF);
//		f.calculF.set(f.aangulaire).multLocal(dt * 0.5f);
//		f.vangulaire.addLocal(f.calculF);
//
//		// update accel
//		f.lastAccel.set(f.acceleration);
//		f.lastAangulaire.set(f.aangulaire);
	}

	@Override
	public void updateForce(long instant, long dt) {
		//force -> accel
		//https://pixelastic.github.io/pokemonorbigdata/
		
		//for each force, decompose them into linear & angular
		System.out.println("Free updateForce! "+f.forces.size());
		Vector3d sumLinear = new Vector3d(0,0,0);
		Vector3d sumAngular = new Vector3d(0,0,0);
		Vector3d fpos = f.position;
		Vector3d vectDir = new Vector3d();
		for(int i=0; i<f.forces.size(); i++){
			vectDir.set(fpos).subtractLocal(f.pointApplicationForce.get(i)).normalizeLocal();
			if(vectDir.lengthSquared() == 0){
//				System.out.println("onlylinear : "+f.forces.get(i));
				sumLinear.addLocal(f.forces.get(i));
			}else{
				double length = f.forces.get(i).length();
				double dot = vectDir.dot(f.forces.get(i));
				sumLinear.addLocal(f.forces.get(i).mult(dot/length));
//				System.out.println("linear : "+vectDir.mult(dot));
				if(dot < length){ //+epsilon?
					Vector3d angularVect = f.forces.get(i).cross(vectDir);
					angularVect.multLocal((length-dot)/length);
//					System.out.println("angular : "+angularVect+" "+f.forces.get(i)+" cross "+vectDir+" mult "+((length-dot)/length));
					
					//recompose angular into linear if combine
					// note: it should not occur on a free-flight object.
					double maxLength = Math.min(length, sumAngular.length());
					if(maxLength>0){
						double percent = angularVect.normalize().dot(sumAngular.normalize());
						if(percent < 0){
//							System.out.println("addLinear : "+f.forces.get(i).mult(-2*percent*(length-dot)/length));
							sumLinear.addLocal(f.forces.get(i).mult(-2*percent*(length-dot)/length));
						}
					}
					sumAngular.addLocal(angularVect);
				}
			}
		}
//		System.out.println("linear final : "+sumLinear);
//		System.out.println("angular final : "+sumAngular);
		if(f.landed) f.acceleration.set(0,0,0);
		else{
			f.acceleration.set(f.positionGravite).subtractLocal(fpos);
			double dist = f.acceleration.length();
			f.acceleration.normalizeLocal().multLocal((f.constanteGraviteMasse/(dist*dist*f.mass)));
		}
		System.out.println("JFF grav : "+f.acceleration);
		System.out.println("JFF sumLinear : "+sumLinear);
		sumLinear.divideLocal(f.mass);
		System.out.println("JFF sumLinear : "+sumLinear);
		f.acceleration.addLocal(sumLinear);
		sumAngular.divideLocal(f.getIntertiaMoment());
		f.aangulaire.set(sumAngular);

//		f.acceleration.set(0,0,0);
//		for (Vector3d force : f.forces) {
//			f.acceleration.addLocal(force);
//		}
//		System.out.println("sumForces : " + f.acceleration);
//		
//		//decomposition en linéaire et angulaire
//		//TODO: need position of forces to do that?
//		// (or not as thay can be decomposed by linear force and angular force)
//		
//		//Angular force: just transpose them to angular acceleration
//		f.aangulaire.set(0,0,0);
//		for (Vector3d force : f.angularForces) {
//			f.aangulaire.addLocal(force);
//		}
//		System.out.println("f.aangulaire="+f.aangulaire);
//		f.aangulaire.divideLocal(f.getIntertiaMoment());
//		
//		//for each force : OF dot F1F2 => lineaire
//		//the remaining is rot with OF/2 as center
//		// => decompose remaining rotation force as lineaire and angular
//		
//		f.acceleration.divideLocal((float)f.mass);
		System.out.println("JFF accel : "+ f.acceleration);
		System.out.println("JFF aaccel : "+ f.aangulaire);
		
	}

	@Override
	public void addCollisionPoint(Vector3d pointCollision, int idx, Forme fOpposite, int idxOpposite) {
		f.joint = new JointPonctuel(f, pointCollision, idx, fOpposite, idxOpposite);
	}

	@Override
	public void gotoCollision(int pointIdx, Vector3d pointObj) {
		//check diff
		Vector3d worldPos = new Vector3d();
		f.transfoMatrix.mult(f.points.get(pointIdx), worldPos);
		System.out.println("  worldPos="+worldPos+" to pointObj="+pointObj);
		f.position.addLocal(new Vector3d(pointObj.subtract(worldPos)));
		f.transfoMatrix.setTranslation(f.position);
	}

	@Override
	public void removeCollisionPoint(Vector3d pointCollision, int idx) {
	}

	@Override
	public void gotoCollision(int pointIdx, PlaneD planeObj) {
		Vector3d worldPos = new Vector3d();
		f.transfoMatrix.mult(f.points.get(pointIdx), worldPos);
		Vector3d pointObj = planeObj.getClosestPoint(worldPos);
		System.out.println("  worldPos="+worldPos+" to pointObj="+pointObj);
		f.position.addLocal(new Vector3d(pointObj.subtract(worldPos)));
		f.transfoMatrix.setTranslation(f.position);
	}

	@Override
	public void gotoCollision(Vector3d localTA, Vector3d localTB, Vector3d localTC, Vector3d worldObj) {
		PlaneD p = new PlaneD();
		Vector3d tempA = f.transfoMatrix.mult(localTA, new Vector3d());
		Vector3d tempB = f.transfoMatrix.mult(localTB, new Vector3d());
		Vector3d tempC = f.transfoMatrix.mult(localTC, new Vector3d());
		p.setPlanePoints(tempA, tempB, tempC);
		Vector3d worldPos =p.getClosestPoint(worldObj);
		System.out.println("  worldPos="+worldPos+" to pointObj="+worldObj);
		f.position.addLocal(new Vector3d(worldObj.subtract(worldPos)));
		f.transfoMatrix.setTranslation(f.position);
		
		//TODO: faire un peu de rotation en meme temps!
	}
}
