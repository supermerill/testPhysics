package collision;

import com.jme3.math.Ray;
import com.jme3.math.Vector3f;

import old.Forme;

public class CollisionPrediction {
	
	//false if it's not possible now.
	public boolean finded = true;
	public boolean init = false;
	
	
	public long moment;
	public Forme formePoint;
	public int pointIdx;
	public Forme formeTri;
	public int triIdx;
	
	public Vector3f worldTA = new Vector3f();
	public Vector3f worldTB = new Vector3f();
	public Vector3f worldTC = new Vector3f();
	public Vector3f worldP = new Vector3f();
	public Vector3f localTA;
	public Vector3f localTB;
	public Vector3f localTC;
	public Vector3f localP;
	public Ray rayon;
	public Vector3f bestP; //for log only
	
}
