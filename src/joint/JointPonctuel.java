package joint;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

import com.jme3.math.Matrix4f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

import old.Forme;

public class JointPonctuel extends JointRotation {

	//del these if not used.
//	public Forme f1;
//	public int idxPointf1;
//	public Forme f2;
//	public int idxPointf2;
	
	//the point-joint
//	public Vector3f point; //World coord
	public int idxPoint;
	
	
	public JointPonctuel(Forme f, Vector3f pointCollision, int idx) {
		super(f);
		this.pointRotation = pointCollision;
		this.idxPoint = idx;
	}

	@Override
	public void updatePosition(long instant, long dt) {
		super.updatePosition(instant, dt);
		//recaler le solide
		f.position.addLocal(pointRotation.subtract(f.transfoMatrix.mult(new Vector3f(f.points.get(f.points.size()-1)))));
	}
//		//set linear vit from vang
//		Quaternion quaterAdd = new Quaternion().fromAngleAxis(f.vangulaire.length() * dt, f.vangulaire);
//		System.out.println("vangulaire : "+f.vangulaire);
//		System.out.println("pos : "+f.position);
//		System.out.println("point : "+point);
//		System.out.println("f.transfoMatrix.invert().mult(point) : "+f.transfoMatrix.invert().mult(point));
//	
//		Matrix4f newRot = new Matrix4f();
//		newRot.setTransform(Vector3f.ZERO, Vector3f.UNIT_XYZ,
//				quaterAdd.toRotationMatrix());
//		
//		Matrix4f preciseTrsf = new Matrix4f();
//		preciseTrsf.setTransform(f.position.toVec3f(), Vector3f.UNIT_XYZ,
//				f.pangulaire.toRotationMatrix());
//		
//		Vector3f transl = new Vector3f(f.transfoMatrix.invert().mult(point)).mult(1);
////		new Matrix4d(f.transfoMatrix).multNormal(transl, transl);
//		preciseTrsf.multNormal(transl, transl);
//		newRot.m03 = transl.x - newRot.m00 * transl.x - newRot.m01 * transl.y - newRot.m02
//				* transl.z;
//		newRot.m13 = transl.y - newRot.m10 * transl.x - newRot.m11 * transl.y - newRot.m12
//				* transl.z;
//		newRot.m23 = transl.z - newRot.m20 * transl.x - newRot.m21 * transl.y - newRot.m22
//				* transl.z;
//		System.out.println("Translation : "+newRot.toTranslationVector());
//		
//		//f.transfoMatrix.translateVect(newRot.toTranslationVector());
////		System.out.println("previousPs: "+f.position);
//		Vector3f newSpeed = newRot.toTranslationVector();
//		
//		//should be done in update (to check possible collisions...
//		//			f.position.addLocal(newSpeed);
//
//		//recalage sur les points de rotation
//		
////		System.out.println("correctedPs: "+f.position);
////		f.transfoMatrix.set(newRot.multLocal(f.transfoMatrix));
//		Vector3f lastPosPoint = preciseTrsf.mult(new Vector3f(f.points.get(f.points.size()-1)));
//		System.out.println("Point pos: "+preciseTrsf.mult(new Vector3f(f.points.get(f.points.size()-1))));
//		//as it's an integration =>notlinear, discrete (via dt), replace the point at the good place
//
//		preciseTrsf.setTransform(f.position.toVec3f(), Vector3f.UNIT_XYZ,
//				f.pangulaire.toRotationMatrix());
//		
//		Vector3f newPosPoint = preciseTrsf.mult(new Vector3f(f.points.get(f.points.size()-1)));
//		newSpeed.addLocal(lastPosPoint.subtractLocal(newPosPoint));
//		f.position.addLocal(lastPosPoint);
//		//TODO: plus qu'un point
//
//		//utile pour le calcul de collision
//		f.vitesse.set(newSpeed);
//		System.out.println("Jointonctuel : speed now "+newSpeed);
//	}


	@Override
	public void updateForce(long instant, long dt) {
		if(f.landed) return;
		
		//compute sum of force
		Vector3f sumForce = new Vector3f(0,0,0);
		for (Vector3f force : f.forces) {
			sumForce.addLocal(force);
		}

		//create the normal force (via a dot)
		Vector3f normalN = f.position.toVec3f().subtractLocal(pointRotation);
		normalN.normalizeLocal();
		Vector3f normal = normalN.mult(normalN.dot(sumForce));
		System.out.println("normaleN : "+normalN);
		System.out.println("length/totalLength : "+normalN.dot(sumForce) +" / "+sumForce.length());
		System.out.println("normale : "+normal+" (length="+normal.length());

		//create rotationVector
		Vector3f rotVector = normalN.cross(sumForce.normalize()).mult(sumForce.length() - normal.length());
		rotVector.divideLocal((float)(f.roundBBRayon*f.roundBBRayon*f.mass/3));
		f.aangulaire.addLocal(rotVector);
		System.out.println("sum : "+sumForce);
		System.out.println("cross : "+normalN.cross(sumForce.normalize()));
		System.out.println("crossN : "+normalN.cross(sumForce.normalize()).mult(sumForce.length() - normal.length()));
		System.out.println("rotVector : "+rotVector);
		System.out.println("f.aangulaire : "+f.aangulaire);
		}

	@Override
	public void addCollisionPoint(Vector3f pointCollision, int idx) {
		System.out.println("joint ponctuel:@"+pointRotation+" : add "+pointCollision);
		f.joint = new JointPose(f);
		f.joint.addCollisionPoint(pointRotation, idxPoint);
		f.joint.addCollisionPoint(pointCollision, idx);
		
	}

	@Override
	public Collection<Integer> getIdx() {
		return Arrays.asList(idxPoint);
	}

}
