package joint;

import jme3Double.Vector3d;
import old.Forme;

import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

public class JointFreeFlight extends Joint {

	public JointFreeFlight(Forme f) {
		super(f);
	}
	
	@Override
	public void updateVitesse(long instant, long dtms) {
		float dts = dtms*0.001f;

		// update linear speed
		f.calculF.set(f.lastAccel).multLocal(dts * 0.5f);
		f.vitesse.addLocal(f.calculF);
		f.calculF.set(f.acceleration).multLocal(dts * 0.5f);
		f.vitesse.addLocal(f.calculF);
		System.out.println(f.name+"JFF now speed = " + f.vitesse+" from accel "+f.acceleration+" and lastaccel="+f.lastAccel);

		// update angular speed
		f.calculF.set(f.lastAangulaire).multLocal(dts * 0.5f);
		f.vangulaire.addLocal(f.calculF);
		f.calculF.set(f.aangulaire).multLocal(dts * 0.5f);
		f.vangulaire.addLocal(f.calculF);
		System.out.println(f.name+"JFF now aspeed = " + f.vangulaire+" from aAcel="+f.aangulaire+" & "+f.lastAangulaire);
	}
	
	@Override
	public void updatePosition(long instant, long dtms) {
		float dts = dtms*0.001f;

		// update linear position
		f.calculF.set(f.vitesse).multLocal(dts);
		f.position.addLocal(f.calculF);
		System.out.println(f.name+"JFF now pos = " + f.vitesse);

		// update angular position
		f.pangulaire.fromRotationMatrix(f.transfoMatrix.toRotationMatrix());
		Quaternion quaterAdd = new Quaternion().fromAngleAxis(f.vangulaire.length() * dts, f.vangulaire);
		f.pangulaire.multLocal(quaterAdd);
		f.pangulaire.normalizeLocal();
		System.out.println(f.name+"JFF now apos = " + f.vangulaire);
		

		f.transfoMatrix.setTransform(f.position.toVec3fLocal(f.calculF), Vector3f.UNIT_XYZ,
				f.pangulaire.toRotationMatrix());
		
	}

	@Override
	public void updateForce(long instant, long dt) {
		//force -> accel
		//https://pixelastic.github.io/pokemonorbigdata/
		
		//for each force, decompose them into linear & angular
		System.out.println("Free updateForce! "+f.forces.size());
		Vector3f sumLinear = new Vector3f(0,0,0);
		Vector3f sumAngular = new Vector3f(0,0,0);
		Vector3f fpos = f.position.toVec3f();
		Vector3f vectDir = new Vector3f();
		for(int i=0; i<f.forces.size(); i++){
			vectDir.set(fpos).subtractLocal(f.pointApplicationForce.get(i)).normalizeLocal();
			if(vectDir.lengthSquared() == 0){
//				System.out.println("onlylinear : "+f.forces.get(i));
				sumLinear.addLocal(f.forces.get(i));
			}else{
				float length = f.forces.get(i).length();
				float dot = vectDir.dot(f.forces.get(i));
				sumLinear.addLocal(f.forces.get(i).mult(dot/length));
//				System.out.println("linear : "+vectDir.mult(dot));
				if(dot < length){ //+epsilon?
					Vector3f angularVect = f.forces.get(i).cross(vectDir);
					angularVect.multLocal((length-dot)/length);
//					System.out.println("angular : "+angularVect+" "+f.forces.get(i)+" cross "+vectDir+" mult "+((length-dot)/length));
					
					//recompose angular into linear if combine
					// note: it should not occur on a free-flight object.
					float maxLength = Math.min(length, sumAngular.length());
					if(maxLength>0){
						float percent = angularVect.normalize().dot(sumAngular.normalize());
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
			float dist = f.acceleration.length();
			f.acceleration.normalizeLocal().multLocal((float)(f.constanteGraviteMasse/(dist*dist*f.mass)));
		}
		System.out.println("JFF grav : "+f.acceleration);
		System.out.println("JFF sumLinear : "+sumLinear);
		sumLinear.divideLocal((float)f.mass);
		System.out.println("JFF sumLinear : "+sumLinear);
		f.acceleration.addLocal(sumLinear);
		sumAngular.divideLocal(f.getIntertiaMoment());
		f.aangulaire.set(sumAngular);

//		f.acceleration.set(0,0,0);
//		for (Vector3f force : f.forces) {
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
//		for (Vector3f force : f.angularForces) {
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
	public void addCollisionPoint(Vector3f pointCollision, int idx, Forme fOpposite, int idxOpposite) {
		f.joint = new JointPonctuel(f, pointCollision, idx, fOpposite, idxOpposite);
	}

	@Override
	public void gotoCollision(int pointIdx, Vector3f pointObj) {
		//check diff
		Vector3f worldPos = new Vector3f();
		f.transfoMatrix.mult(f.points.get(pointIdx), worldPos);
		
		//PROBLEME: si exactement superposé, on ne sais plus trop de quel coté est cahque objet.
		// SOLUTION: utiliser la normale du point pour désambuguiser ou s'arreter juste avant.
		worldPos.addLocal(f.normales.get(pointIdx).mult(0.000001f)); //TODO verifier
		
		System.out.println("JFF p->p   worldPos="+worldPos+" to pointObj="+pointObj);
		f.position.addLocal(new Vector3d(pointObj.subtract(worldPos)));
		f.transfoMatrix.setTranslation(f.position.toVec3f());
	}

	@Override
	public void removeCollisionPoint(Vector3f pointCollision, int idx) {
	}

	@Override
	public void gotoCollision(int pointIdx, Plane planeObj) {
		Vector3f worldPos = new Vector3f();
		f.transfoMatrix.mult(f.points.get(pointIdx), worldPos);
		Vector3f pointObj = planeObj.getClosestPoint(worldPos);
		
		//PROBLEME: si exactement superposé, on ne sais plus trop de quel coté est cahque objet.
		// SOLUTION: utiliser la normale du point pour désambuguiser ou s'arreter juste avant.
		worldPos.addLocal(planeObj.getNormal().mult(0.00001f));

		System.out.println("JFF p->plane  worldPos="+worldPos+" to pointObj="+pointObj+", normale:"+planeObj.getNormal());
		f.position.addLocal(new Vector3d(pointObj.subtract(worldPos)));
		f.transfoMatrix.setTranslation(f.position.toVec3f());
	}

	@Override
	public void gotoCollision(Vector3f localTA, Vector3f localTB, Vector3f localTC, Vector3f worldObj) {
		Plane p = new Plane();
		Vector3f tempA = f.transfoMatrix.mult(localTA, new Vector3f());
		Vector3f tempB = f.transfoMatrix.mult(localTB, new Vector3f());
		Vector3f tempC = f.transfoMatrix.mult(localTC, new Vector3f());
		p.setPlanePoints(tempA, tempB, tempC);
		Vector3f worldPos =p.getClosestPoint(worldObj);
		
		//PROBLEME: si exactement superposé, on ne sais plus trop de quel coté est cahque objet.
		// SOLUTION: utiliser la normale du point pour désambuguiser ou s'arreter juste avant.
		worldPos.addLocal(p.getNormal().mult(0.00001f)); // bon sens... tout le temps?
		
		System.out.println("JFF tri->p   worldPos="+worldPos+" to pointObj="+worldObj);
		f.position.addLocal(new Vector3d(worldObj.subtract(worldPos)));
		f.transfoMatrix.setTranslation(f.position.toVec3f());
		
		//TODO: faire un peu de rotation en meme temps!
	}
}
