package old;

import com.jme3.math.Vector3f;

public class Triangle {

	public Triangle(Forme f, int i, int j, int k) {
		a=i;b=j;c=k;
		float ij = f.points.get(i).distance(f.points.get(j));
		float jk = f.points.get(j).distance(f.points.get(k));
		float ki = f.points.get(k).distance(f.points.get(i));
		bbRound = Math.max(Math.max(ij,jk),ki);
		center = f.points.get(i).add(f.points.get(j)).addLocal(f.points.get(k)).divideLocal(3);
	}

	public int a,b,c;
	public float bbRound; // roundbounding box
	public Vector3f center;
	public String toString(){return a+","+b+","+c;}
}
