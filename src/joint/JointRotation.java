package joint;

import old.Forme;

import com.jme3.math.Matrix4f;
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
		
		//should be done in update (to check possible collisions...
		//			f.position.addLocal(newSpeed);

		//recalage sur les points de rotation
		
//		System.out.println("correctedPs: "+f.position);
//		f.transfoMatrix.set(newRot.multLocal(f.transfoMatrix));
//		Vector3f lastPosPoint = preciseTrsf.mult(new Vector3f(f.points.get(f.points.size()-1)));
		System.out.println("JointRotation Point: "+f.transfoMatrix.mult(new Vector3f(f.points.get(f.points.size()-1))));
//		System.out.println("JointRotation Point lastPosPoint: "+lastPos);
//		System.out.println("JointRotation Point curentPosPoint: "+lastPosPoint);
//		System.out.println("JointRotation Point diff: "+lastPosPoint.distance(lastPos));
//		lastPos.set(lastPosPoint);
		//as it's an integration =>notlinear, discrete (via dt), replace the point at the good place

//		preciseTrsf.setTransform(f.position.toVec3f(), Vector3f.UNIT_XYZ,
//				f.pangulaire.toRotationMatrix());
		
//		Vector3f newPosPoint = preciseTrsf.mult(new Vector3f(f.points.get(f.points.size()-1)));
//		newSpeed.addLocal(lastPosPoint.subtractLocal(newPosPoint));
		//f.position.addLocal(lastPosPoint);
		//TODO: plus qu'un point

		//utile pour le calcul de collision
		f.vitesse.set(newSpeed.mult(10)); ///???? *10?
		System.out.println("JointRotation : speed now "+f.vitesse+" on "+f);
		System.out.println("JointRotation : speed displacement this turn "+f.vitesse.mult(dts).length());
	}
//	Vector3f lastPos = new Vector3f();

}
