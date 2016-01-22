package old;

public class Vect {
	public Vect(int i, int j, int k) {
		x=i;y=j;z=k;
	}
	public Vect(Vect v) {
		this(v.x, v.y, v.z);
	}
	int x;
	int y;
	int z;
}
