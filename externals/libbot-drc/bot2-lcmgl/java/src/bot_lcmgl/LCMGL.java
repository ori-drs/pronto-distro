package bot_lcmgl;

import java.io.*;

import lcm.lcm.LCM;

public class LCMGL 
{
    ByteArrayOutputStream _bouts = new ByteArrayOutputStream();
    DataOutputStream _outs = new DataOutputStream(_bouts);
    String _channel = "LCMGL";
    LCM _lcm = null;
    String _name = "unnamed";
    int _scene = 0;
    int _sequence = 0;

    final static int LCMGL_BEGIN         = 4;
    final static int LCMGL_END           = 5;
    final static int LCMGL_VERTEX3F      = 6;
    final static int LCMGL_VERTEX3D      = 7;
    final static int LCMGL_COLOR3F       = 8;
    final static int LCMGL_COLOR4F       = 9;
    final static int LCMGL_POINTSIZE     = 10;
    final static int LCMGL_ENABLE        = 11;
    final static int LCMGL_DISABLE       = 12;
    final static int LCMGL_BOX           = 13;
    final static int LCMGL_CIRCLE        = 14;
    final static int LCMGL_LINE_WIDTH    = 15;
    final static int LCMGL_NOP           = 16;
    final static int LCMGL_VERTEX2D      = 17;
    final static int LCMGL_VERTEX2F      = 18;
    final static int LCMGL_TEXT          = 19;
    final static int LCMGL_DISK          = 20;
    final static int LCMGL_TRANSLATED    = 21;
    final static int LCMGL_ROTATED       = 22;
    final static int LCMGL_LOAD_IDENTITY = 23;
    final static int LCMGL_PUSH_MATRIX   = 24;
    final static int LCMGL_POP_MATRIX    = 25;
    final static int LCMGL_RECT          = 26;
    final static int LCMGL_TEXT_LONG     = 27;
    final static int LCMGL_NORMAL3F      = 28;
    final static int LCMGL_SCALEF        = 29;
    final static int LCMGL_MULT_MATRIXF  = 30;
    final static int LCMGL_MULT_MATRIXD  = 31;
    final static int LCMGL_MATERIALF     = 32;

    public LCMGL(LCM lcm, String name)
    {
        _lcm = lcm;
        _name = name;
    }

    private synchronized void add0(int cmd)
    {
        try { _outs.writeByte(cmd); } catch(IOException xcp) {}
    }

    private synchronized void add1i(int cmd, int arg)
    {
        try { 
            _outs.writeByte(cmd); 
            _outs.writeInt(arg);
        } catch(IOException xcp) {}
    }

    private synchronized void add1f(int cmd, float arg)
    {
        try { 
            _outs.writeByte(cmd); 
            _outs.writeFloat(arg);
        } catch(IOException xcp) {}
    }

    private synchronized void add2f(int cmd, float x, float y)
    {
        try { 
            _outs.writeByte(cmd); 
            _outs.writeFloat(x);
            _outs.writeFloat(y);
        } catch(IOException xcp) {}
    }

    private synchronized void add2d(int cmd, double x, double y)
    {
        try { 
            _outs.writeByte(cmd); 
            _outs.writeDouble(x);
            _outs.writeDouble(y);
        } catch(IOException xcp) {}
    }

    private synchronized void add3f(int cmd, float x, float y, float z)
    {
        try { 
            _outs.writeByte(cmd); 
            _outs.writeFloat(x);
            _outs.writeFloat(y);
            _outs.writeFloat(z);
        } catch(IOException xcp) {}
    }

    private synchronized void add3d(int cmd, double x, double y, double z)
    {
        try { 
            _outs.writeByte(cmd); 
            _outs.writeDouble(x);
            _outs.writeDouble(y);
            _outs.writeDouble(z);
        } catch(IOException xcp) {}
    }

    private synchronized void add4f(int cmd, float x, float y, float z, float w)
    {
        try { 
            _outs.writeByte(cmd); 
            _outs.writeFloat(x);
            _outs.writeFloat(y);
            _outs.writeFloat(z);
            _outs.writeFloat(w);
        } catch(IOException xcp) {}
    }

    private synchronized void add4d(int cmd, double x, double y, double z, double w)
    {
        try { 
            _outs.writeByte(cmd); 
            _outs.writeDouble(x);
            _outs.writeDouble(y);
            _outs.writeDouble(z);
            _outs.writeDouble(w);
        } catch(IOException xcp) {}
    }

    public synchronized void switchBuffers()
    {
        byte[] b = _bouts.toByteArray();
        bot_lcmgl.data_t msg = new bot_lcmgl.data_t();
        msg.name = _name;
        msg.scene = _scene;
        msg.sequence = _sequence;
        msg.data = b;
        msg.datalen = b.length;

        _lcm.publish(_channel, msg);

        _scene++;
        _sequence++;
        _bouts.reset();
    }

    public synchronized void glBegin(int mode) { add1i(LCMGL_BEGIN, mode); }
    public synchronized void glEnd() { add0(LCMGL_END); }

    public synchronized void glEnable(int cap) { add1i(LCMGL_ENABLE, cap); }
    public synchronized void glDisable(int cap) { add1i(LCMGL_DISABLE, cap); }

    public synchronized void glPushMatrix() { add0(LCMGL_PUSH_MATRIX); }
    public synchronized void glPopMatrix() { add0(LCMGL_POP_MATRIX); }
    public synchronized void glLoadIdentity() { add0(LCMGL_LOAD_IDENTITY); }
    public synchronized void glMultMatrixf(float f[]) {
        try {
            _outs.writeByte(LCMGL_MULT_MATRIXF);
            for(int i=0; i<16; i++)
                _outs.writeFloat(f[i]);
        } catch(IOException xcp) {}
    }
    public synchronized void glMultMatrixd(double f[]) {
        try {
            _outs.writeByte(LCMGL_MULT_MATRIXF);
            for(int i=0; i<16; i++)
                _outs.writeDouble(f[i]);
        } catch(IOException xcp) {}
    }

    public synchronized void glPointSize(float size) { add1f(LCMGL_POINTSIZE, size); }
    public synchronized void glLineWidth(float size) { add1f(LCMGL_LINE_WIDTH, size); }

    public synchronized void glColor3f(float r, float g, float b) { add3f(LCMGL_COLOR3F, r, g, b); }
    public synchronized void glColor4f(float r, float g, float b, float a) { add4f(LCMGL_COLOR4F, r, g, b, a); }

    public synchronized void glScalef(float x, float y, float z) { add3f(LCMGL_SCALEF, x, y, z); }
    public synchronized void glTranslated(double x, double y, double z) { add3d(LCMGL_TRANSLATED, x, y, z); }
    public synchronized void glRotated(double theta, double x, double y, double z) { add4d(LCMGL_ROTATED, theta, x, y, z); }

    public synchronized void glNormal3f(float x, float y, float z) { add3f(LCMGL_NORMAL3F, x, y, z); }

    public synchronized void glVertex2d(double x, double y) { add2d(LCMGL_VERTEX2D, x, y); }
    public synchronized void glVertex2f(float x, float y) { add2f(LCMGL_VERTEX2F, x, y); }

    public synchronized void glVertex3d(double x, double y, double z) { add3d(LCMGL_VERTEX3D, x, y, z); }
    public synchronized void glVertex3f(float x, float y, float z) { add3f(LCMGL_VERTEX3F, x, y, z); }

    public synchronized void circle(double x, double y, double z, double r) {
        try {
            _outs.writeByte(LCMGL_CIRCLE); 
            _outs.writeDouble(x);
            _outs.writeDouble(y);
            _outs.writeDouble(z);
            _outs.writeFloat((float)r);
        } catch(IOException xcp) {}
    }

    public synchronized void disk(double x, double y, double z, double r_in, double r_out) {
        try {
            _outs.writeByte(LCMGL_DISK); 
            _outs.writeDouble(x);
            _outs.writeDouble(y);
            _outs.writeDouble(z);
            _outs.writeFloat((float)r_in);
            _outs.writeFloat((float)r_out);
        } catch(IOException xcp) {}
    }
    
    public static final int TEXT_DROP_SHADOW = 1;
    public static final int TEXT_JUSTIFY_LEFT = 2;
    public static final int TEXT_JUSTIFY_RIGHT = 4;
    public static final int TEXT_JUSTIFY_CENTER = 8;
    public static final int TEXT_ANCHOR_LEFT = 16;
    public static final int TEXT_ANCHOR_RIGHT = 32;
    public static final int TEXT_ANCHOR_TOP = 64;
    public static final int TEXT_ANCHOR_BOTTOM = 128;
    public static final int TEXT_ANCHOR_HCENTER = 256;
    public static final int TEXT_ANCHOR_VCENTER = 512;
    public static final int TEXT_NORMALIZED_SCREEN_COORDINATES = 1024;
    public static final int TEXT_MONOSPACED = 2048;    
    
    public synchronized void text(double x, double y, double z, String text) {
        this.text(x, y, z, text, 0, 
                TEXT_DROP_SHADOW | 
                TEXT_JUSTIFY_CENTER |
                TEXT_ANCHOR_HCENTER |
                TEXT_ANCHOR_VCENTER);
    }
    
    public synchronized void text(double x, double y, double z, String text,
            int font, int flags) {
        try {
            _outs.writeByte(LCMGL_TEXT_LONG);
            _outs.writeInt(font);
            _outs.writeInt(flags);
            _outs.writeDouble(x);
            _outs.writeDouble(y);
            _outs.writeDouble(z);
            _outs.writeInt(text.length());
            _outs.writeBytes(text);
        } catch(IOException xcp) {}
    }

    public static void main(String args[])
    {
        LCM lcm = LCM.getSingleton();
        LCMGL lcmgl = new LCMGL(lcm, "Java test");

        // constants taken from javax.media.opengl.GL
        int GL_LINE_LOOP = 2; 
        int GL_LINES = 1;

        double theta = 0;
        double x = 0;
        double y = 0;
        double z = 0;
        double t = 0;
        while(true) {
            lcmgl.glPushMatrix();
            lcmgl.glRotated(theta, 0, 0, 1);
            lcmgl.glTranslated(x, y, z);
            for(int scale=1; scale<=2; scale++) {
                lcmgl.glPushMatrix();
                lcmgl.glScalef(scale, scale, scale);

                lcmgl.glColor3f(scale * 0.5f, scale * 0.5f, scale * 0.5f);

                lcmgl.glBegin(GL_LINE_LOOP);
                lcmgl.glVertex3f(-1, -1, -1);
                lcmgl.glVertex3f(-1,  1, -1);
                lcmgl.glVertex3f( 1,  1, -1);
                lcmgl.glVertex3f( 1, -1, -1);
                lcmgl.glEnd();

                lcmgl.glBegin(GL_LINE_LOOP);
                lcmgl.glVertex3f(-1, -1,  1);
                lcmgl.glVertex3f(-1,  1,  1);
                lcmgl.glVertex3f( 1,  1,  1);
                lcmgl.glVertex3f( 1, -1,  1);
                lcmgl.glEnd();

                lcmgl.glBegin(GL_LINES);
                lcmgl.glVertex3f(-1, -1, -1);
                lcmgl.glVertex3f(-1, -1,  1);
                lcmgl.glVertex3f(-1,  1, -1);
                lcmgl.glVertex3f(-1,  1,  1);
                lcmgl.glVertex3f( 1,  1, -1);
                lcmgl.glVertex3f( 1,  1,  1);
                lcmgl.glVertex3f( 1, -1, -1);
                lcmgl.glVertex3f( 1, -1,  1);
                lcmgl.glEnd();

                lcmgl.glPopMatrix();
            }
            lcmgl.glPopMatrix();
            lcmgl.switchBuffers();

            try { Thread.sleep(30); } catch (InterruptedException xcp) { }
            t += 0.1;
            x = Math.cos(t);
            y = Math.sin(t);
            z = 0;
            theta = 0.1 * t * 180 / Math.PI;
        }
    }

}
