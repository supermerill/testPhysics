package v2.joint;

import v2.Forme;
import jme3Double.Matrix4d;
import jme3Double.PlaneD;
import jme3Double.Quaterniond;
import jme3Double.Vector3d;

public abstract class JointRotation extends Joint {

	//the point-joint
	public Vector3d pointWRotation = new Vector3d(0,0,0); //World coord
	public Vector3d pointLRotation = new Vector3d(0,0,0); //Local coord
	
	
	public JointRotation(Forme f) {
		super(f);
	}
	
//	public static Vector3d getLinearFromRotation(Vector3d angular, Vector3d origin, Vector3d point, long dtms){
//		float dts = dtms*0.001f;
//		//set linear vit from vang
//		Quaternion quaterAdd = new Quaternion().fromAngleAxis(angular.length() * dts, angular);
//		
//		Matrix4f newRot = new Matrix4f();
//		newRot.setTransform(Vector3d.ZERO, Vector3d.UNIT_XYZ, quaterAdd.toRotationMatrix());
//		
//		Matrix4f preciseTrsf = new Matrix4f();
//		preciseTrsf.setTransform(rotationP, Vector3d.UNIT_XYZ, f.pangulaire.toRotationMatrix());
//		
//		Vector3d transl = new Vector3d(f.transfoMatrix.invert().mult(pointWRotation)).mult(1);
////		new Matrix4d(f.transfoMatrix).multNormal(transl, transl);
//		preciseTrsf.multNormal(transl, transl);
//		newRot.m03 = transl.x - newRot.m00 * transl.x - newRot.m01 * transl.y - newRot.m02
//				* transl.z;
//		newRot.m13 = transl.y - newRot.m10 * transl.x - newRot.m11 * transl.y - newRot.m12
//				* transl.z;
//		newRot.m23 = transl.z - newRot.m20 * transl.x - newRot.m21 * transl.y - newRot.m22
//				* transl.z;
//		System.out.println("JointRotation Translation : "+newRot.toTranslationVector());
//		
//		//f.transfoMatrix.translateVect(newRot.toTranslationVector());
////		System.out.println("previousPs: "+f.position);
//		Vector3d newSpeed = newRot.toTranslationVector();
//		
//
//		//vitesse lineaire, pour garder le centre de rotation à peu près où il faut.
//		f.vitesse.set(newSpeed.mult(1)); ///???? *10?
//
//		
//		
//		return null;
//	}
	

	@Override
	public void updatePosition(long instant, long dtms) {
		System.out.println("forme UpdatePos "+f +"@"+f.position);
		float dts = dtms*0.001f;
		//set linear vit from vang
		Quaterniond quaterAdd = new Quaterniond().fromAngleAxis(f.vangulaire.length() * dts, f.vangulaire);
		System.out.println("JointRotation vangulaire : "+f.vangulaire);
		
		Matrix4d newRot = new Matrix4d();
		newRot.setTransform(Vector3d.ZERO, Vector3d.UNIT_XYZ, quaterAdd.toRotationMatrix());
		
		Matrix4d preciseTrsf = new Matrix4d();
		preciseTrsf.setTransform(f.position, Vector3d.UNIT_XYZ, f.pangulaire.toRotationMatrix());
		
		Vector3d transl = new Vector3d(f.transfoMatrix.invert().mult(pointWRotation)).mult(1);
//		new Matrix4d(f.transfoMatrix).multNormal(transl, transl);
		preciseTrsf.multNormal(transl, transl);
		newRot.m03 = transl.x - newRot.m00 * transl.x - newRot.m01 * transl.y - newRot.m02
				* transl.z;
		newRot.m13 = transl.y - newRot.m10 * transl.x - newRot.m11 * transl.y - newRot.m12
				* transl.z;
		newRot.m23 = transl.z - newRot.m20 * transl.x - newRot.m21 * transl.y - newRot.m22
				* transl.z;
		System.out.println("JointRotation Translation : "+newRot.toTranslationVector());
		
		//f.transfoMatrix.translateVect(newRot.toTranslationVector());
//		System.out.println("previousPs: "+f.position);
		Vector3d newSpeed = newRot.toTranslationVector();
		

		//vitesse lineaire, pour garder le centre de rotation à peu près où il faut.
		f.vitesse.set(newSpeed.mult(1)); ///???? *10?
		System.out.println("JointRotation : speed now "+f.vitesse+" on "+f);
		System.out.println("JointRotation : speed displacement this turn "+f.vitesse.mult(dts).length());
		System.out.println("forme End UpdatePos "+f +"@"+f.position);
	}
	
	//TODO: allow to return "can't collide"
	//TODO: refactor the nbIter / bestDist to end when we are not going any better => revert and return false
//	Vector3d lastPos = new Vector3d();
	@Override
	public void gotoCollision(int pointIdx, PlaneD planeObj) {
		System.out.println("gotoCollision "+pointIdx+" => "+planeObj);
		//get world pos
		Vector3d WP = new Vector3d();
		Vector3d LP = f.points.get(pointIdx);
		f.transfoMatrix.mult(LP, WP);
		
		Vector3d WR = pointWRotation;
		Vector3d LR = pointLRotation; //f.transfoMatrix.invert().mult(pointWRotation);

		

		System.out.println("BeforeRecalage : " + pointWRotation + ".distance("
				+ f.transfoMatrix.mult(pointLRotation) 
				+ ") ="+pointWRotation.distance(f.transfoMatrix.mult(new Vector3d(pointLRotation))));
		//it's hard to me to figure it out exactly in 1 pass right now,
		//i just incrementally adjust the position of the object
		double vitesse = Math.abs(f.vangulaire.cross(LP).addLocal(f.vitesse).dot(planeObj.getClosestPoint(WP).subtract(WP).normalizeLocal()));
//		float distance = FastMath.sqrt(rayonObjectif.distanceSquared(WP));
		double distance = planeObj.pseudoDistance(WP);
		double dts = distance / vitesse; // distance / speed => m / m/s => s
//		while(FastMath.abs(distance)>0.00005f){
//			System.out.println("WP="+WP+", VP="+VP+", dist="+distance+" vitesse="+vitesse+" dts="+dts);
//			rotateALittle(dts, LR);
//			//relance
//			f.transfoMatrix.mult(LP, WP);
//			distance = planObjectif.pseudoDistance(WP);
//			dts = 0.99f*distance / vitesse;
//		}
		
		System.out.println("LR="+LR+" WR="+WR+" and now L->WR = "+f.transfoMatrix.mult(LR));
		while(distance < 0 ){
			System.out.println("try to recover from a bad collision: "+distance);
			System.out.println("PPl INIT WP="+WP+", dist="+distance+" vitesse="+vitesse+" dts="+dts);
			rotateALittle(dts, LR);
			//relance
			f.transfoMatrix.mult(LP, WP);
			distance = planeObj.pseudoDistance(WP);
			dts*=2;
		}
		int nbIter=0;
		double bestDist = distance*1.01f;
		while((distance>0.00005f || distance < 0  )&& bestDist > Math.abs(distance)){
			System.out.println("PPl WP="+WP+", dist="+distance+" vitesse="+vitesse+" dts="+dts);
			System.out.println("LR="+LR+" WR="+WR+" and now L->WR = "+f.transfoMatrix.mult(LR));
			rotateALittle(dts, LR);
			//relance
			f.transfoMatrix.mult(LP, WP);
			distance = planeObj.pseudoDistance(WP);
			dts = (distance<0?2.14f:0.9f)*distance / vitesse;
			if(nbIter>10) dts = dts*0.314f;
			nbIter++;
		}
		System.out.println("end recalage with precision@"+Math.abs(distance));
		System.out.println("LR="+LR+" WR="+WR+" and now L->WR = "+f.transfoMatrix.mult(LR));
		System.out.println("WP: "+WP+", obj : " + planeObj.getClosestPoint(WP));
		System.out.println("AfterRecalage : " + pointWRotation + ".distance("
				+ f.transfoMatrix.mult(pointLRotation) 
				+ ") ="+pointWRotation.distance(f.transfoMatrix.mult(new Vector3d(pointLRotation))));
	}

	@Override
	public void gotoCollision(Vector3d localTA, Vector3d localTB, Vector3d localTC, Vector3d worldObj) {
		System.out.println("gotoCollision "+localTA+localTB+localTC+" => "+worldObj);
		PlaneD myPlane = new PlaneD();
		Vector3d tempA = f.transfoMatrix.mult(localTA, new Vector3d());
		Vector3d tempB = f.transfoMatrix.mult(localTB, new Vector3d());
		Vector3d tempC = f.transfoMatrix.mult(localTC, new Vector3d());
		myPlane.setPlanePoints(tempA, tempB, tempC);
		Vector3d tempP = myPlane.getClosestPoint(worldObj);
		System.out.println("example: "+tempP+" => "+worldObj);

		Vector3d LR = pointLRotation;//f.transfoMatrix.invert().mult(pointWRotation);
		
		//it's hard to me to figure it out exactly in 1 pass right now,
		//i just incrementally adjust the position of the object
		System.out.println("vitesse: "+f.vitesse+" angulVitesse="+f.vangulaire.cross(tempP)+" direction voulue: "+worldObj.subtract(tempP).normalizeLocal());
		double vitesse = Math.abs(f.vangulaire.cross(tempP).addLocal(f.vitesse).dot(worldObj.subtract(tempP).normalizeLocal()));
//		float distance = FastMath.sqrt(rayonObjectif.distanceSquared(WP));
		double distance = myPlane.pseudoDistance(worldObj);
		double dts = distance / vitesse; // distance / speed => m / m/s => s
//		while(FastMath.abs(distance)>0.00005f){
//			System.out.println("WP="+WP+", VP="+VP+", dist="+distance+" vitesse="+vitesse+" dts="+dts);
//			rotateALittle(dts, LR);
//			//relance
//			f.transfoMatrix.mult(LP, WP);
//			distance = planObjectif.pseudoDistance(WP);
//			dts = 0.99f*distance / vitesse;
//		}
		System.out.println("distance = "+distance);
		while(distance<0){
			System.out.println("try to recover from a bad collision: "+distance);
			rotateALittle(dts, LR);
			//relance
			f.transfoMatrix.mult(localTA, tempA);
			f.transfoMatrix.mult(localTB, tempB);
			f.transfoMatrix.mult(localTC, tempC);
			myPlane.setPlanePoints(tempA, tempB, tempC);
			distance = myPlane.pseudoDistance(worldObj);
			dts*=2;
		}
		dts = 0.5f*distance / vitesse;
		int nbIter=0;
		double bestDist = distance*1.01f;
		while(distance>0.00005f || distance < 0 ){
			System.out.println("PlP myPlane="+myPlane+", dist="+distance+" vitesse="+vitesse+" dts="+dts);
			rotateALittle(dts, LR);
			//relance
			f.transfoMatrix.mult(localTA, tempA);
			f.transfoMatrix.mult(localTB, tempB);
			f.transfoMatrix.mult(localTC, tempC);
			myPlane.setPlanePoints(tempA, tempB, tempC);
			distance = myPlane.pseudoDistance(worldObj);
			dts = (distance<0?2.14f:0.9f)*distance / vitesse;
			if(nbIter>10) dts = dts*0.314f;
			nbIter++;
		}
		System.out.println("end recalage with precision@"+Math.abs(distance));
		System.out.println("WP="+myPlane.getClosestPoint(worldObj)+", obj : " + worldObj);
	}
	
	@Override
	public void gotoCollision(int pointIdx, Vector3d pointObj) {
		System.out.println("gotoCollision "+pointIdx+" => "+pointObj);
		//get world pos
		Vector3d WP = new Vector3d();
		Vector3d LP = f.points.get(pointIdx);
		f.transfoMatrix.mult(LP, WP);
		
		Vector3d VP = pointObj;
		Vector3d WR = pointWRotation;
		Vector3d LR = pointLRotation; //f.transfoMatrix.invert().mult(pointRotation);
		Vector3d WR2VP = VP.subtract(WR);
		//now we need to find the point PP
		// PP is the position of WP in the line WR,VP 
//		Ray rayonObjectif = new Ray(WR, VP.subtract(WR));
		PlaneD planObjectif = new PlaneD();
		planObjectif.setOriginNormal(WR, WR2VP.cross(f.vangulaire).normalizeLocal());
//		Plane planObjectif2 = new Plane();
//		planObjectif2.setOriginNormal(WR, WR2VP.cross(f.vangulaire));

		//TODOAFTER
		//as it's hard to me to figure it our right now, i just incrementally adjust the position of the object
		double vitesse = Math.abs(f.vangulaire.cross(LP).addLocal(f.vitesse).dot(pointObj.subtract(WP).normalizeLocal()));
//		float distance = FastMath.sqrt(rayonObjectif.distanceSquared(WP));
		double distance = planObjectif.pseudoDistance(WP);
		double dts = distance / vitesse; // distance / speed => m / m/s => s
//		while(FastMath.abs(distance)>0.00005f){
//			System.out.println("WP="+WP+", VP="+VP+", dist="+distance+" vitesse="+vitesse+" dts="+dts);
//			rotateALittle(dts, LR);
//			//relance
//			f.transfoMatrix.mult(LP, WP);
//			distance = planObjectif.pseudoDistance(WP);
//			dts = 0.99f*distance / vitesse;
//		}
		while(distance>0.00005f || distance < 0 ){
			System.out.println("try to recover from a bad collision: "+distance);
			rotateALittle(dts, LR);
			f.transfoMatrix.mult(LP, WP);
			distance = planObjectif.pseudoDistance(WP);
			dts*=2;
		}
		while(distance>0.00005f || distance < 0 ){
			System.out.println("WP="+WP+", VP="+VP+", dist="+distance+" vitesse="+vitesse+" dts="+dts);
			rotateALittle(dts, LR);
			//relance
			f.transfoMatrix.mult(LP, WP);
			distance = planObjectif.pseudoDistance(WP);
			dts = (distance<0?2.14f:0.99f)*distance / vitesse;
		}
		System.out.println("end recalage with precision@"+Math.abs(distance));
		System.out.println("WP: "+WP+", obj : " + planObjectif.getClosestPoint(WP));
	}

	private void rotateALittleF(double dts, Vector3d localCenterOfRotation){
		//linear
		f.calcul1.set(f.vitesse).multLocal(dts);
		f.position.addLocal(f.calcul1);
		System.out.println("MOVE : " + f.calcul1 + " on "+f.position + " => "+f.position.add(new Vector3d(f.calcul1)));
		//angular
		Quaterniond quaterAdd = new Quaterniond().fromAngleAxis(f.vangulaire.length() * dts, f.vangulaire);
		System.out.println("ROTATE : " + f.vangulaire.mult(dts)+" on "+f.pangulaire+" => "+f.pangulaire.mult(quaterAdd));
		f.pangulaire.multLocal(quaterAdd);
		f.pangulaire.normalizeLocal();
		//matrix
		f.transfoMatrix.setTransform(f.position, Vector3d.UNIT_XYZ,
			f.pangulaire.toRotationMatrix());
		//recalage au cas ou : keep le centre de rotation FIXE a tout prix
		f.position.addLocal(pointWRotation.subtract(f.transfoMatrix.mult(localCenterOfRotation)));
		f.transfoMatrix.setTranslation(f.position);
	}
	private void rotateALittle(double dts, Vector3d localCenterOfRotation){
		//linear
		f.calcul1.set(f.vitesse).multLocal(dts);
		f.position.addLocal(f.calcul1);
		System.out.println("MOVE : " + f.calcul1 + " on "+f.position + " => "+f.position.add(f.calcul1));
		//angular
		Quaterniond quaterAdd = new Quaterniond().fromAngleAxis(f.vangulaire.length() * dts, new Vector3d(f.vangulaire));
		Quaterniond quaterPos = new Quaterniond(f.pangulaire);
		System.out.println("ROTATE : " + f.vangulaire.mult(dts)+" on "+quaterPos+" => "+quaterPos.mult(quaterAdd));
		quaterPos.multLocal(quaterAdd);
		quaterPos.normalizeLocal();
		f.pangulaire.set((float)quaterPos.x, (float)quaterPos.y, (float)quaterPos.z, (float)quaterPos.w);
		
		//matrix
		f.transfoMatrix.setTransform(f.position, Vector3d.UNIT_XYZ,
			f.pangulaire.toRotationMatrix());
		//recalage : keep le centre de rotation FIXE a tout prix
		f.position.addLocal(pointWRotation.subtract(f.transfoMatrix.mult(localCenterOfRotation)));
		f.transfoMatrix.setTranslation(f.position);
	}

}
