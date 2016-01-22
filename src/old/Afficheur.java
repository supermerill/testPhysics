package old;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.ArrayList;

import javax.swing.JComponent;
import javax.swing.JFrame;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix4f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

public class Afficheur extends JComponent {

	ArrayList<Forme> formes = new ArrayList<>();
	Color c = Color.BLACK;
	
	int keyPress;

	public static void main(String[] args) {
		JFrame fenetre = new JFrame();
		final Afficheur view = new Afficheur();

		Forme f = new Forme();
		f.points.add(new Vector3d(-15, -15, 0));
		f.points.add(new Vector3d(15, -15, 0));
		f.points.add(new Vector3d(-15, 15, 0));
		f.points.add(new Vector3d(15, 15, 0));
		f.triangles.add(new Forme.Triangle(0, 1, 2));
		f.triangles.add(new Forme.Triangle(1, 2, 3));
		f.pangulaire = new Quaternion().fromAngleAxis(FastMath.PI*40f/180, Vector3f.UNIT_Z);
		f.landed = false;
		view.formes.add(f);
		
		
		//sol

		f = new Forme();
		f.points.add(new Vector3d(-200, -200, 0));
		f.points.add(new Vector3d(200, -200, 0));
		f.points.add(new Vector3d(-200, -220, 0));
		f.points.add(new Vector3d(200, -220, 0));
		f.triangles.add(new Forme.Triangle(0, 1, 2));
		f.triangles.add(new Forme.Triangle(1, 2, 3));
		view.formes.add(f);

		fenetre.add(view);
		fenetre.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		fenetre.setSize(800, 500);
		fenetre.setVisible(true);
		
		fenetre.addKeyListener(new KeyListener() {
			
			@Override
			public void keyTyped(KeyEvent e) {
				// TODO Auto-generated method stub
				
			}
			
			@Override
			public void keyReleased(KeyEvent e) {
				if(view.keyPress == e.getKeyCode()){
					view.keyPress = 0;
				}
			}
			
			@Override
			public void keyPressed(KeyEvent e) {
				view.keyPress = e.getKeyCode();
			}
		});

		new Thread() {
			public void run() {
				long previousMs = System.currentTimeMillis();
				long nextMs = System.currentTimeMillis();
				while (true) {
					
					view.repaint();
					nextMs += 100;
					// sleep for 13ms
					if (System.currentTimeMillis() < nextMs) {
						try {
							sleep(nextMs - System.currentTimeMillis());
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
					}
					long currentMs = System.currentTimeMillis();
//					System.out.println("finish sleep: "+currentMs+" =?= "+nextMs);
					int diff = (int)(currentMs-previousMs);
//					System.out.println("diff: "+diff +" = "+currentMs+" - "+previousMs);
					//calculate
					for(Forme f : view.formes){
						f.update(diff);
					}
					

					for(Forme f : view.formes){
						if(!f.landed)
						for(Forme f2 : view.formes){
							if(f2.landed){
								if(f.checkCollision(f2)){
									f.vitesse.set(Vector3d.ZERO);
									f.acceleration.set(Vector3d.ZERO);
									f.landed = true;
								}
							}
						}
					}
					
					//apply gravity
					Vector3d force = new Vector3d(0,0,0);
					if(view.keyPress == KeyEvent.VK_Q){
						force.set(20000,20000,0);
					}
					if(view.keyPress == KeyEvent.VK_D){
						force.set(-20000,20000,0);
					}
					if(view.keyPress == KeyEvent.VK_Z){
						force.set(0,200000,0);
					}
					for(Forme f : view.formes){
						f.applyForce(force);
					}
					previousMs = currentMs;
				}
			}
		}.start();
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
		for (Forme forme : formes) {
			//create transformation matrix
			for (Forme.Triangle tri : forme.triangles) {

//				System.out.println("m = "+forme.points.get(tri.a).toVec3f());
//				System.out.println("m = "+forme.transfoMatrix.mult(forme.points.get(tri.a).toVec3f(), null));
				
				Vector3f m = forme.transfoMatrix.mult(forme.points.get(tri.a).toVec3f(), null);
				Vector3f n = forme.transfoMatrix.mult(forme.points.get(tri.b).toVec3f(), null);
				Vector3f e = forme.transfoMatrix.mult(forme.points.get(tri.c).toVec3f(), null);
				g.drawLine(maxX + (int) (m.x + m.z / 2), maxY
						- (int) (m.y + m.z / 2), maxX + (int) (n.x + n.z / 2),
						maxY - (int) (n.y + n.z / 2));
				g.drawLine(maxX + (int) (e.x + e.z / 2), maxY
						- (int) (e.y + e.z / 2), maxX + (int) (n.x + n.z / 2),
						maxY - (int) (n.y + n.z / 2));
				g.drawLine(maxX + (int) (m.x + m.z / 2), maxY
						- (int) (m.y + m.z / 2), maxX + (int) (e.x + e.z / 2),
						maxY - (int) (e.y + e.z / 2));
			}
		}

	}

}
