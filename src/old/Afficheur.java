package old;

import java.awt.Color;
import java.awt.Graphics;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.util.ArrayList;

import javax.swing.JComponent;
import javax.swing.JFrame;

import collision.CollisionUpdater;

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

		Forme f = new Forme("TRAP");
		f.points.add(new Vector3f(-15, -15, -4));
		f.points.add(new Vector3f(15, -15, -6));
		f.points.add(new Vector3f(-15, 15, -5));
		f.points.add(new Vector3f(15, 15, -3));
		f.points.add(new Vector3f(0, 0, 5));
		f.triangles.add(f.new Triangle(0, 1, 2));
		f.triangles.add(f.new Triangle(1, 2, 3));
		f.triangles.add(f.new Triangle(0, 1, 4));
		f.triangles.add(f.new Triangle(1, 2, 4));
		f.triangles.add(f.new Triangle(2, 3, 4));
		f.triangles.add(f.new Triangle(3, 0, 4));
		f.roundBBRayon = 30;
		f.pangulaire = new Quaternion().fromAngleAxis(FastMath.PI*32f/180, Vector3f.UNIT_Z);
		f.acceleration.set(0,-0.00981f,0);
		f.lastAccel.set(f.acceleration);
		f.landed = false;
		f.doNotFaceCenter();
		view.formes.add(f);
		
		
		//sol

		f = new Forme("SOL");
		f.points.add(new Vector3f(-200, 10, -100)); //0
		f.points.add(new Vector3f(-200, 10, 100)); //1
		f.points.add(new Vector3f(200, 10, 100)); //2
		f.points.add(new Vector3f(200, 10, -100)); //3
		f.points.add(new Vector3f(-200, -10, -100)); //4
		f.points.add(new Vector3f(-200, -10, 100)); //5
		f.points.add(new Vector3f(200, -10, 100)); //6
		f.points.add(new Vector3f(200, -10, -100)); //7
		f.triangles.add(f.new Triangle(1,0,4));
		f.triangles.add(f.new Triangle(4,5,1));
		f.triangles.add(f.new Triangle(2,1,5));
		f.triangles.add(f.new Triangle(5,6,2));
		f.triangles.add(f.new Triangle(3,0,4));
		f.triangles.add(f.new Triangle(4,7,3));
		f.triangles.add(f.new Triangle(0, 1, 2));
		f.triangles.add(f.new Triangle(2, 3, 0));
		f.triangles.add(f.new Triangle(4, 5, 6));
		f.triangles.add(f.new Triangle(6, 7, 4));
		f.triangles.add(f.new Triangle(2,6,7));
		f.triangles.add(f.new Triangle(7,3,2));



		f.points.add(new Vector3f(0, 0, 0)); //8
		f.points.add(new Vector3f(0, 0, 0)); //9
//		
		f.points.add(new Vector3f(-33, 10, 0)); //10
		f.points.add(new Vector3f(20, 10, 20)); //11
		f.points.add(new Vector3f(20, 10, -20)); //12
		f.points.add(new Vector3f(0, 55, 0)); //13
//		f.points.add(new Vector3f(-133, 10, 0)); //10
//		f.points.add(new Vector3f(-80, 10, 20)); //11
//		f.points.add(new Vector3f(-80, 10, -20)); //12
//		f.points.add(new Vector3f(-100, 100, 0)); //13
		f.roundBBRayon = 250;
		f.position.set(0,-200,0);
		f.triangles.add(f.new Triangle(10, 13, 11));
		f.triangles.add(f.new Triangle(11, 13, 12));
		f.triangles.add(f.new Triangle(12, 13, 10));
		f.doNotFaceCenter();
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
				CollisionUpdater updater = new CollisionUpdater();
				updater.allFormes = view.formes;
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
					System.out.println("NEW TURN ---------------------------------------------------------------------------------");
					//calculate
					for(Forme f : view.formes){
//						f.update(5*diff/2);
					}
					
					updater.updateCollision(currentMs, diff/10);

//					for(Forme f : view.formes){
//						if(!f.landed)
//						for(Forme f2 : view.formes){
//							if(f2.landed){
//								CollisionUpdater.predictRayCollision(f, f2, currentMs);
//								CollisionMobileSol collision = f.checkCollision(f2);
//								if(collision != null){
//									//TODO: placer f tel que le point qui a touch� soit pile 
//									//dans le plan du triangle touch�
//									collision.placeMobile();
//									
//									//TODO: spliter le triangle en cr�ant un nouveau point "identique � l'autre
//									//TODO: "lier" les deux objets via un joint tournant (frottements infini)..oupa
//									
//									//TODO: continuer � le faire r�agir � la pesanteur
//									// => modifier l'algo de maj si li�
//									//		faire le calcul des forces => ajout des r�sultantes => acceleration
//									//		si objet li� �  1 point => si dans la dir de la somme des forces => le garder sinon le retirer et revenir au mode normal
//									//			faire une rotation libre
//									//		si objet li� � deux points & dans la meme dir que force, 
//									//			faire un rotation sur un axe fixe
//									//			en fait, avec une somme des resultantes, pas besoin de deuxi�me cas, ni de troisi�me.
//									
//									f.vitesse.set(Vector3f.ZERO);
//									f.acceleration.set(Vector3f.ZERO);
//									f.lastAccel.set(Vector3f.ZERO);
//									f.landed = true;
//								}
//							}
//						}
//					}
					
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
						if(!f.landed) f.applyForce(force);
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
			System.out.println(" =Forme@"+forme.position+" : "+forme.transfoMatrix);
			//create transformation matrix
			for (Forme.Triangle tri : forme.triangles) {

				System.out.println("m = "+forme.points.get(tri.a));
				System.out.println("m = "+forme.transfoMatrix.mult(forme.points.get(tri.a), null));
				
				Vector3f m = forme.transfoMatrix.mult(forme.points.get(tri.a), null);
				Vector3f n = forme.transfoMatrix.mult(forme.points.get(tri.b), null);
				Vector3f e = forme.transfoMatrix.mult(forme.points.get(tri.c), null);
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
