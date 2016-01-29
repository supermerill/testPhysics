package old;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.ArrayList;
import java.util.Random;

import javax.swing.JComponent;
import javax.swing.JFrame;

import jme3Double.Vector3d;

import collision.CollisionUpdater;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix4f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

public class TestJointPoseAfficheur extends JComponent {

	ArrayList<Forme> formes = new ArrayList<>();
	Color c = Color.BLACK;

	int keyPress;

	public static void main(String[] args) {
		JFrame fenetre = new JFrame();
		final TestJointPoseAfficheur view = new TestJointPoseAfficheur();

		Forme f = new Forme("TRAP");
		f.points.add(new Vector3f(-100, 20, -50)); // 0
		f.points.add(new Vector3f(-100, 20, 50)); // 1
		f.points.add(new Vector3f(100, 20, 50)); // 2
		f.points.add(new Vector3f(100, 20, -50)); // 3
		f.points.add(new Vector3f(-100, -20, -50)); // 4
		f.points.add(new Vector3f(-100, -20, 50)); // 5
		f.points.add(new Vector3f(100, -20, 50)); // 6
		f.points.add(new Vector3f(100, -20, -50)); // 7
		f.triangles.add(f.new Triangle(1, 0, 4));
		f.triangles.add(f.new Triangle(4, 5, 1));
		f.triangles.add(f.new Triangle(2, 1, 5));
		f.triangles.add(f.new Triangle(5, 6, 2));
		f.triangles.add(f.new Triangle(3, 0, 4));
		f.triangles.add(f.new Triangle(4, 7, 3));
		f.triangles.add(f.new Triangle(0, 1, 2));
		f.triangles.add(f.new Triangle(2, 3, 0));
		f.triangles.add(f.new Triangle(4, 5, 6));
		f.triangles.add(f.new Triangle(6, 7, 4));
		f.triangles.add(f.new Triangle(2, 6, 7));
		f.triangles.add(f.new Triangle(7, 3, 2));
		f.roundBBRayon = 150;
		// f.position.set(-50,0,0);
		// f.pangulaire = new Quaternion().fromAngleAxis(FastMath.PI*32f/180,
		// Vector3f.UNIT_Z);
		f.acceleration.set(0.000000f, -0.00000981f, 0);
		// f.vangulaire.set(0,0.001f,0); //1 rotation toute les 5sec
		f.physicUpdate = true;
		f.landed = false;
		f.doNotFaceCenter();

//		 f.jointPose.points.add(new Vector3f(0,-30,0));
//		 f.jointPose.points.add(new Vector3f(30,-30,0));
//		 f.jointPose.points.add(new Vector3f(-30,-30,0));
//		 f.jointPose.points.add(new Vector3f(0,-30,30));
//		 f.jointPose.points.add(new Vector3f(0,-30,-30));

		// f.jointPose.points.add(new Vector3f(30,-20,30));
		// f.jointPose.points.add(new Vector3f(30,-20,-30));
		// f.jointPose.points.add(new Vector3f(-30,-20,30));
		// f.jointPose.points.add(new Vector3f(-30,-20,-30));

		// f.jointPose.points.add(new Vector3f(100,-30,0));
		// f.jointPose.points.add(new Vector3f(-100,-30,0));
		// f.jointPose.points.add(new Vector3f(0,-30,100));
		// f.jointPose.points.add(new Vector3f(0,-30,-100));
		// f.jointPose.points.add(new Vector3f(100,-30,100));
		// f.jointPose.points.add(new Vector3f(-100,-30,100));
		// f.jointPose.points.add(new Vector3f(100,-30,-100));
		// f.jointPose.points.add(new Vector3f(-100,-30,-100));

//		 f.jointPose.points.add(new Vector3f(40,0,-30));
//		 f.jointPose.points.add(new Vector3f(-30,0,40));
//		 f.jointPose.points.add(new Vector3f(-30,0,-30));

		// f.jointPose.points.add(new Vector3f(-10,0,0));
		// f.jointPose.points.add(new Vector3f(100,0,0));

//		f.jointPose.points.add(new Vector3f(-100, 0, -50));
//		f.jointPose.points.add(new Vector3f(0, 0, -30));
//		f.jointPose.points.add(new Vector3f(-30, 0, -30));
//		f.jointPose.points.add(new Vector3f(-3, 0, 30));
//		f.jointPose.points.add(new Vector3f(-30, 0, 0));
//		f.jointPose.points.add(new Vector3f(-20, 0, 15));
		

//		f.jointPose.points.add(new Vector3f(-10, 0, -30));
//		f.jointPose.points.add(new Vector3f(-30, 0, -30));
//		f.jointPose.points.add(new Vector3f(-30, 0, 10));
//		f.jointPose.points.add(new Vector3f(10, 0, -30));
//		f.jointPose.points.add(new Vector3f(-4, 0, -10.1f));

//		f.jointPose.points.add(new Vector3f(10, 0, 30));
//		f.jointPose.points.add(new Vector3f(30, 0, 30));
//		f.jointPose.points.add(new Vector3f(30, 0, 10));
//		f.jointPose.points.add(new Vector3f(10, 0, 30));
//		f.jointPose.points.add(new Vector3f(4, 0, 0.1f));

//		f.jointPose.points.add(new Vector3f(9.63876f, 0, 3.0678787f));
//		f.jointPose.points.add(new Vector3f(12.811367f, 0, -8.452932f));
//		f.jointPose.points.add(new Vector3f(19.352798f, 0, 11.159054f));
//		f.jointPose.points.add(new Vector3f(-6.111166f, 0, -11.166454f));
//		f.jointPose.points.add(new Vector3f(26.28289f, 0, -6.854574f));
		

//		f.jointPose.points.add(new Vector3f(-14.720682f, 0, 2.3897095f));
//		f.jointPose.points.add(new Vector3f(-19.071083f, 0, -10.783894f));
//		f.jointPose.points.add(new Vector3f(-15.972529f, 0, -6.0288467f));
//		f.jointPose.points.add(new Vector3f(-6.6198025f, 0, 16.227226f));
//		f.jointPose.points.add(new Vector3f(8.292912f, 0, -11.612946f));

//f.jointPose.points.add(new Vector3f(29.061073f, 0, 16.655365f));
//f.jointPose.points.add(new Vector3f(22.77488f, 0, -17.025723f));
//f.jointPose.points.add(new Vector3f(32.93412f, 0, 47.316376f));
//f.jointPose.points.add(new Vector3f(-8.480434f, 0, 9.62236f));
//f.jointPose.points.add(new Vector3f(33.12845f, 0, -25.385738f));

		Random rand = new Random();
		for(int i=0;i<5;i++){
			f.jointPose.points.add(new Vector3f((rand.nextFloat()*100)-25, 0, (rand.nextFloat()*100)-25));
		}
		
//		f.jointPose.points.add(new Vector3f(-1.3086681f, 0, -3.8920822f));
//		f.jointPose.points.add(new Vector3f(-8.117588f, 0, 23.260162f));
//		f.jointPose.points.add(new Vector3f(18.827335f, 0, 18.136639f));
//		f.jointPose.points.add(new Vector3f(-14.779891f, 0, 29.894077f));
//		f.jointPose.points.add(new Vector3f(14.058125f, 0, 25.960533f));
		

		f.position.set(0, 20, 0);
		
		f.jointPose.f = f;
		f.forces.add(new Vector3f(0, -200, 0));

		f.jointPose.updateForce(0, 0);

		view.formes.add(f);

		fenetre.add(view);
		fenetre.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		fenetre.setSize(800, 500);
		fenetre.setVisible(true);
	}

	@Override
	public void paintComponent(Graphics g) {
		super.paintComponent(g);
		paintInside(g);
	}

	private void paintInside(Graphics g) {
		// g.setColor(Color.RED);
		// g.fillOval(0, 0, 400, 400);

		int maxY = getHeight() / 2;
		int maxX = getWidth() / 2;

		int taille = 10;

		g.setColor(c);
		System.out.println("===DRAW===");
//		System.out.println("maxX = "+maxX+"maxY = "+maxY);
		for (Forme forme : formes) {
			System.out.println("p1 = "+forme.jointPose.point1);
			System.out.println("p2 = "+forme.jointPose.point2);
			// System.out.println(" =Forme@"+forme.position+" : "+forme.transfoMatrix);
			// create transformation matrix
			g.setColor(Color.WHITE);
			for (Forme.Triangle tri : forme.triangles) {

				// System.out.println("m = "+forme.points.get(tri.a));
				// System.out.println("m = "+forme.transfoMatrix.mult(forme.points.get(tri.a),
				// null));

				Vector3f m = forme.transfoMatrix.mult(forme.points.get(tri.a), null);
				Vector3f n = forme.transfoMatrix.mult(forme.points.get(tri.b), null);
				Vector3f e = forme.transfoMatrix.mult(forme.points.get(tri.c), null);
				g.drawLine(maxX + (int) (m.x + m.y / 2), maxY - (int) (m.z + m.y / 2), maxX + (int) (n.x + n.y / 2),
						maxY - (int) (n.z + n.y / 2));
				g.drawLine(maxX + (int) (e.x + e.y / 2), maxY - (int) (e.z + e.y / 2), maxX + (int) (n.x + n.y / 2),
						maxY - (int) (n.z + n.y / 2));
				g.drawLine(maxX + (int) (m.x + m.y / 2), maxY - (int) (m.z + m.y / 2), maxX + (int) (e.x + e.y / 2),
						maxY - (int) (e.z + e.y / 2));
			}

			g.setColor(Color.BLACK);
			for (Vector3f pc : forme.jointPose.points) {
				g.fillOval((int) (maxX + pc.x + pc.y / 2) - 2, (int) (maxY - pc.z - pc.y / 2) - 2, 5, 5);
			}
			g.setColor(Color.CYAN);
			g.fillOval((int) (maxX + 0) - 2, (int) (maxY - 0) - 2, 5, 5);
			{

				g.setColor(Color.MAGENTA);
				Vector3f m = new Vector3f(0, 0, 0);
				Vector3f n = forme.jointPose.normale2Draw.mult(100);
				g.drawLine(maxX + (int) (m.x + m.y / 2), maxY - (int) (m.z + m.y / 2), maxX + (int) (n.x + n.y / 2),
						maxY - (int) (n.z + n.y / 2));
				g.setColor(Color.GREEN);
				n = forme.jointPose.normale2Draw2.mult(100);
				g.drawLine(maxX + (int) (m.x + m.y / 2), maxY - (int) (m.z + m.y / 2), maxX + (int) (n.x + n.y / 2),
						maxY - (int) (n.z + n.y / 2));
				g.setColor(Color.CYAN);
				n = forme.jointPose.normale2Draw3.mult(100);
				g.drawLine(maxX + (int) (m.x + m.y / 2), maxY - (int) (m.z + m.y / 2), maxX + (int) (n.x + n.y / 2),
						maxY - (int) (n.z + n.y / 2));
			}

			g.setColor(Color.ORANGE);
			for (int i = 0; i < forme.jointPose.normales.size(); i++) {
				Vector3f m = forme.jointPose.points.get(i);
				Vector3f n = forme.jointPose.normales.get(i).add(forme.jointPose.points.get(i));
				g.drawLine(maxX + (int) (m.x + m.y / 2), maxY - (int) (m.z + m.y / 2), maxX + (int) (n.x + n.y / 2),
						maxY - (int) (n.z + n.y / 2));
			}
			g.setColor(Color.MAGENTA);
			{
				Vector3f m = forme.jointPose.forceResultante.add(forme.jointPose.pointPivot);
				Vector3f n = forme.jointPose.pointPivot;
				g.drawLine(maxX + (int) (m.x + m.y / 2), maxY - (int) (m.z + m.y / 2), maxX + (int) (n.x + n.y / 2),
						maxY - (int) (n.z + n.y / 2));

				Vector3f pc = forme.jointPose.point1;
//				System.out.println("draw point1 : "+pc+ ": "+(int)( (maxX + pc.x + pc.y / 2) - 2)+", "+(int)( (maxY - pc.z - pc.y / 2) - 2)+", "+5+", "+5);
//				System.out.println("draw point1 : "+pc+ ": "+(int)( (pc.x + pc.y / 2) - 2)+", "+(int)( (pc.z - pc.y / 2) - 2));
//				System.out.println("draw point1 : "+(int)( ((maxY - pc.y) - (pc.z / 2)) - 2)+" = "+(maxY - pc.y)+" - "+( (pc.z / 2) - 2));
//				System.out.println("draw point1 : "+(maxY - pc.y)+" = "+maxY+" - "+pc.y);
				g.fillOval((int) (maxX + pc.x + pc.y / 2) - 2, (int) (maxY - pc.z - pc.y / 2) - 2, 5, 5);
				g.setColor(Color.GREEN);
				pc = forme.jointPose.point2;
				g.fillOval((int) (maxX + pc.x + pc.y / 2) - 2, (int) (maxY - pc.z - pc.y / 2) - 2, 5, 5);
				g.setColor(Color.BLUE);
				n = forme.jointPose.point1.add(forme.jointPose.point2.subtract(forme.jointPose.point1).mult(10000));
				m = forme.jointPose.point1.add(forme.jointPose.point2.subtract(forme.jointPose.point1).mult(-10000));
				g.drawLine(maxX + (int) (m.x + m.y / 2), maxY - (int) (m.z + m.y / 2), maxX + (int) (n.x + n.y / 2),
						maxY - (int) (n.z + n.y / 2));

			}

			g.setColor(Color.RED);
			if (forme.vitesse.lengthSquared() != 0) {
				Vector3f m = forme.vitesse.mult(1000).addLocal(forme.position.toVec3f());
				Vector3f n = forme.position.toVec3f();
				g.drawLine(maxX + (int) (m.x + m.y / 2), maxY - (int) (m.z + m.y / 2), maxX + (int) (n.x + n.y / 2),
						maxY - (int) (n.z + n.y / 2));
			}
			if (forme.vangulaire.lengthSquared() != 0) {
				Vector3f m = forme.vangulaire.mult(1000).addLocal(forme.position.toVec3f());
				Vector3f n = forme.position.toVec3f();
				// draw the rotaion vector?
				// g.drawLine(maxX + (int) (m.x + m.y / 2), maxY
				// - (int) (m.z + m.y / 2), maxX + (int) (n.x + n.y / 2),
				// maxY - (int) (n.z + n.y / 2));
				for (Vector3f point : forme.points) {
					m = forme.transfoMatrix.mult(point, null);
					Vector3f om = m.subtract(forme.position.toVec3f());
					Vector3f rotVect = forme.vangulaire.mult(1000).crossLocal(om);
					n = rotVect.addLocal(m);
					// compound vith velocity?
					// n.addLocal(forme.vitesse.mult(1000));
					// draw each point of rotation
					g.drawLine(maxX + (int) (m.x + m.y / 2), maxY - (int) (m.z + m.y / 2),
							maxX + (int) (n.x + n.y / 2), maxY - (int) (n.z + n.y / 2));

				}

			}
		}

	}

}
