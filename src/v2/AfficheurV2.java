//package v2;
//import java.awt.Color;
//import java.awt.Graphics;
//import java.util.ArrayList;
//
//import javax.swing.JComponent;
//import javax.swing.JFrame;
//
//import com.jme3.math.Vector3f;
//import com.jme3.scene.Geometry;
//import com.jme3.scene.Mesh;
//import com.jme3.scene.VertexBuffer.Type;
//import com.jme3.util.BufferUtils;
//
//
//public class AfficheurV2 extends JComponent{
//
//	ArrayList<Geometry> formes = new ArrayList<>();
//	
//	
//	static Mesh createCustomMesh1(){
//		Mesh mesh = new Mesh();
//		Vector3f [] vertices = new Vector3f[4];
//		vertices[0] = new Vector3f(0,0,0);
//		vertices[1] = new Vector3f(30,0,0);
//		vertices[2] = new Vector3f(0,30,0);
//		vertices[3] = new Vector3f(30,30,0);
////		Vector2f[] texCoord = new Vector2f[4];
////		texCoord[0] = new Vector2f(0,0);
////		texCoord[1] = new Vector2f(1,0);
////		texCoord[2] = new Vector2f(0,1);
////		texCoord[3] = new Vector2f(1,1);
//		int [] indexes = { 2,0,1, 1,3,2 };
//		mesh.setBuffer(Type.Position, 3, BufferUtils.createFloatBuffer(vertices));
////		mesh.setBuffer(Type.TexCoord, 2, BufferUtils.createFloatBuffer(texCoord));
//		mesh.setBuffer(Type.Index,    3, BufferUtils.createIntBuffer(indexes));
//		mesh.updateBound();
//		
//		Geometry geo = new Geometry("OurMesh", mesh);
//		
//		return mesh;
//	}
//	
//	public static void main(String[] args) {
//		JFrame fenetre = new JFrame();
//		AfficheurV2 view = new AfficheurV2();
//		
//		Mesh mesh = createCustomMesh1();
//
//		fenetre.add(view);
//		fenetre.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
//		fenetre.setSize(800,500);
//		fenetre.setVisible(true);
//	}
//	
//	@Override
//	public void paintComponent(Graphics g) {
//		super.paintComponent(g);
//		paintInside(g);
//	}
//
//	private void paintInside(Graphics g) {
////		g.setColor(Color.RED);
////		g.fillOval(0, 0, 400, 400);
//
//		int maxY = getHeight()/2;
//		int maxX = getWidth()/2;
//		
//		int taille = 10;
//		
//		g.setColor(Color.BLACK);
//		for(Forme forme : formes){
//			for(Forme.Triangle tri : forme.triangles){
//				Vect3f m = forme.points.get(tri.a);
//				Vect3f n = forme.points.get(tri.b);
//				g.drawLine(maxX+(int)(m.x+m.z/2), maxY-(int)(m.y+m.z/2), maxX+(int)(n.x+n.z/2), maxY-(int)(n.y+n.z/2));
//				m = forme.points.get(tri.b);
//				n = forme.points.get(tri.c);
//				g.drawLine(maxX+(int)(m.x+m.z/2), maxY-(int)(m.y+m.z/2), maxX+(int)(n.x+n.z/2), maxY-(int)(n.y+n.z/2));
//				m = forme.points.get(tri.c);
//				n = forme.points.get(tri.a);
//				g.drawLine(maxX+(int)(m.x+m.z/2), maxY-(int)(m.y+m.z/2), maxX+(int)(n.x+n.z/2), maxY-(int)(n.y+n.z/2));
//			}
//		}
//		
//	}
//	
//}
