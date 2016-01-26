package joint;

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;

import old.Forme;

public class JointPonctuel extends Joint {

	//del these if not used.
	public Forme f1;
	public int idxPointf1;
	public Forme f2;
	public int idxPointf2;
	
	//the point-joint
	public Vector3f point;
	
	
	public void computeJointForce(Forme f){
		if(f.landed) return;
		int idxP = idxPointf2;
		if(f == f1){
			idxP = idxPointf1;
		}
		
		//compute the sum of all force
		//TODO remove mass... to create accel or add mass to gravity to have a force
//		Vector3f grav = new Vector3f(0,-0.00000981f,0);
		Vector3f grav = new Vector3f(0,-9.81f,0);
		
		
		//create the normal force (via a dot)
		Vector3f normal = f.position.toVec3f().subtractLocal(point);
		System.out.println("normale : "+normal);
//		Vector3f normal = point.subtract(f.position.toVec3f());
		normal.normalizeLocal();
		normal.multLocal(Math.abs(normal.dot(grav)));
		
		//compute the new sum (this vector is _|_ with the normal force)
		Vector3f sumAccel = normal.add(grav);
		System.out.println("grav : "+grav);
		System.out.println("normale : "+normal);
		System.out.println("sum : "+sumAccel);
		System.out.println("dot : "+sumAccel.dot(normal));
		System.out.println("0.0001 : "+0.0001f);
		if(sumAccel.dot(normal) == 0) 
		{
			//exactly at vert. Add some D
			normal.addLocal(new Vector3f(0.000000001f,-0.000000001f, 0.000000001f));
			sumAccel = normal.add(grav);
		}
		assert sumAccel.dot(normal) < 0.0001 : "Error: the normal vector is not _|_ with the resultante";
		
		//add it as rotational force
		float rayon = f.position.toVec3f().distance(point);
		//on a l'acceleration en m/s² en 1 point
		// on veut obtenir l'acceleration en rad/s²
		//or 1 rad = 180° = pi*r metres
		//donc (rad/s²) * pi*r = m/s²
		
		//on a l'acceleration en repère global, pour avoir l'acceleration selon les angles x y et z
		// il faut faire ????
		//perhaps the other way : point.subtractLocal(f.position.toVec3f())
		f.aangulaire.add(sumAccel.cross(f.position.toVec3f().subtractLocal(point)));
		f.vangulaire.set(0,0,0);
		f.vitesse.set(0,0,0);
		f.acceleration.set(0,0,0);
		f1.physicUpdate = false;

		//PAS bonne methode: on veut une rotation autour de Point!
		//donc une combinaison de rotation et translation?
		
		
		//and add a linear force that stop all linear mvt.
		
		//TODO: with friction, split this force into linear and rotational
	}

}