package joint;

import old.Forme;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix4f;
import com.jme3.math.Plane;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

public abstract class JointRotation extends Joint {

	//the point-joint
	public Vector3f pointRotation = new Vector3f(0,0,0); //World coord
	
	
	public JointRotation(Forme f) {
		super(f);
	}

	@Override
	public void updatePosition(long instant, long dtms) {
		float dts = dtms*0.001f;
		//set linear vit from vang
		Quaternion quaterAdd = new Quaternion().fromAngleAxis(f.vangulaire.length() * dts, f.vangulaire);
		System.out.println("JointRotation vangulaire : "+f.vangulaire);
		
		Matrix4f newRot = new Matrix4f();
		newRot.setTransform(Vector3f.ZERO, Vector3f.UNIT_XYZ,
				quaterAdd.toRotationMatrix());
		
		Matrix4f preciseTrsf = new Matrix4f();
		preciseTrsf.setTransform(f.position.toVec3f(), Vector3f.UNIT_XYZ,
				f.pangulaire.toRotationMatrix());
		
		Vector3f transl = new Vector3f(f.transfoMatrix.invert().mult(pointRotation)).mult(1);
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
		Vector3f newSpeed = newRot.toTranslationVector();
		

		//vitesse lineaire, pour garder le centre de rotation à peu près où il faut.
		f.vitesse.set(newSpeed.mult(10)); ///???? *10?
		System.out.println("JointRotation : speed now "+f.vitesse+" on "+f);
		System.out.println("JointRotation : speed displacement this turn "+f.vitesse.mult(dts).length());
	}
//	Vector3f lastPos = new Vector3f();
	
	@Override
	public void gotoCollision(int pointIdx, Vector3f pointObj) {
		//get world pos
		Vector3f WP = new Vector3f();
		Vector3f LP = f.points.get(pointIdx);
		f.transfoMatrix.mult(LP, WP);
		
		Vector3f VP = pointObj;
		Vector3f WR = pointRotation;
		Vector3f LR = f.transfoMatrix.invert().mult(pointRotation);
		Vector3f WR2VP = VP.subtract(WR);
		//now we need to find the point PP
		// PP is the position of WP in the line WR,VP 
//		Ray rayonObjectif = new Ray(WR, VP.subtract(WR));
		Plane planObjectif = new Plane();
		planObjectif.setOriginNormal(WR, WR2VP.cross(f.vangulaire).normalizeLocal());
//		Plane planObjectif2 = new Plane();
//		planObjectif2.setOriginNormal(WR, WR2VP.cross(f.vangulaire));

		//TODOAFTER
		//as it's hard to me to figure it our right now, i just incrementally adjust the position of the object
		float vitesse = f.vangulaire.cross(LP).addLocal(f.vitesse).length();
//		float distance = FastMath.sqrt(rayonObjectif.distanceSquared(WP));
		float distance = planObjectif.pseudoDistance(WP);
		float dts = distance / vitesse; // distance / speed => m / m/s => s
		while(FastMath.abs(distance)>0.00005f){
			System.out.println("WP="+WP+", VP="+VP+", dist="+distance+" vitesse="+vitesse+" dts="+dts);
			rotateALittle(dts, LR);
			//relance
			f.transfoMatrix.mult(LP, WP);
			distance = planObjectif.pseudoDistance(WP);
			dts = 0.99f*distance / vitesse;
		}
		System.out.println("end recalage with precision@"+FastMath.abs(distance));
		System.out.println("WP: "+WP+", obj : " + planObjectif.getClosestPoint(WP));
	}
	
	private void rotateALittle(float dts, Vector3f localCenterOfRotation){
		//linear
		f.calculF.set(f.vitesse).multLocal(dts);
		f.position.addLocal(f.calculF);
		System.out.println("MOVE : " + f.calculF);
		//angular
		Quaternion quaterAdd = new Quaternion().fromAngleAxis(f.vangulaire.length() * dts, f.vangulaire);
		f.pangulaire.multLocal(quaterAdd);
		f.pangulaire.normalizeLocal();
		//matrix
		f.transfoMatrix.setTransform(f.position.toVec3fLocal(f.calculF), Vector3f.UNIT_XYZ,
			f.pangulaire.toRotationMatrix());
		//recalage au cas ou : keep le centre de rotation FIXE a tout prix
		f.position.addLocal(pointRotation.subtract(f.transfoMatrix.mult(localCenterOfRotation)));
		f.transfoMatrix.setTranslation(f.position.toVec3fLocal(f.calculF));
	}

}
